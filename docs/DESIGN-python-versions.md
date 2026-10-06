# Python versions, environments, and the ArduPilot pin

**Raised:** 2026-10-06, contract cards WP1-25, WP2-19, X-06 (Python 3.11+) and
ARCH-31 (ArduPilot version)
**Status:** the guardrail package now states Python 3.11+ (`pyproject.toml`).
The OpenVLA flight environment stays on Python 3.10.20 as a recorded deviation,
which still needs the PI's written acceptance (`docs/PI-DECISIONS-2026-10-06.md`,
Q4 item 2). ArduPilot is pinned to `Copter-4.5.7` in `sitl/setup_sitl.sh`, but
**none of the stored SITL runs flew on that version**, and the WSL tree today
fails the pin check on one nested submodule (see "ArduPilot pin").

Every version number below was read on this PC on 2026-10-06 with
`python --version`, `importlib.metadata.version()`, `pip list` in the WSL
venvs, or `git` in `~/ardupilot`. None is copied from a document.

## What the grant asks for

| Where | Text |
|---|---|
| Policy DSL p1 | "Implementation: Python 3.11+ with Pydantic v2 for both DSL parsing and IR types." |
| Prefix Compiler p1 | "Implementation: Python 3.11+ with Pydantic v2 for the CSP schema." |
| Safety Shield p1 | "Implementation: Python 3.11+ as a ROS 2 node (rclpy)." |
| Stress Testing p1 | "Implementation: Python 3.11+ throughout (harness, scenario authoring, exporters)." |
| Overview p4 (locked decisions) | Implementation language **(non-VLA stack)**: "Python + Pydantic v2 throughout" |
| Overview p5, Safety Shield p7 (open questions) | "ROS 2 distribution. Humble (LTS, Ubuntu 22.04, AI Wings parity), Iron, or Jazzy?" |
| Architecture constraints p3 | ArduPilot inner loop: "Bit-identical between SITL and firmware" |

The "3.11+" lines are on the four **non-VLA** component pages above. The
Overview's locked language decision names no version, and it too is scoped to
the non-VLA stack. The VLA runtime itself (OpenVLA, AerialVLA) is outside both.
The Shield, compiler, and KPI code are inside them wherever they run.

Where the grant is silent, the PI's reference implementation
(`kuanting-vla-uav-guardrail/`) fills the gap: every package there declares
`requires-python = ">=3.11"` (`packages/policy-dsl`, `safety-shield`,
`vlaguard-common`), and the workspace sets ruff `target-version = "py311"` and
mypy `python_version = "3.11"`. Its `policy_dsl` fails to import on 3.10.20
(checked): the shared `vlaguard_common` package it depends on uses
`enum.StrEnum` (`vlaguard_common/manifest.py:11`).

## Decision

1. **The guardrail package declares Python 3.11+.** `pyproject.toml`
   (repo root) packages `guardrail/` only, with `requires-python = ">=3.11"`.
   It takes its version from `VERSION`, and its core dependencies are those of
   `requirements.txt`, plus `jinja2` (see "Open items"). Optional extras:
   `signing` (cryptography, for Ed25519 bundle signatures), `api`
   (fastapi + uvicorn, for `guardrail/api.py`), and `tokens` (tokenizers, for
   the exact OpenVLA token counter in `guardrail/csp.py`). `demo/`,
   `training/`, `tools/`, `sitl/`, and the OpenVLA code are **not** packages.
   Two modules ship but are not promised by an install. `guardrail/vla_bc.py`
   needs torch and `training/` from the source tree.
   `vla_backends.AeroVLABackend` is an unfilled stub (its model load is a
   commented-out TODO); once filled, it will need torch and transformers from
   the VLA env.
2. **The guardrail code must keep importing on 3.10 while vla-real flies,** and
   on 3.12 for the WSL venvs. That means no `tomllib`, `enum.StrEnum`,
   `typing.Self`, `datetime.UTC`, or `except*` in `guardrail/`, and every
   optional dependency is imported behind a guard. `vla-real` imports guardrail
   from the source tree (sys.path), not from an install, so the 3.11 floor in
   `pyproject.toml` does not block it. That is deliberate, and it is the
   deviation below, not something the floor hides.
3. **vla-real stays on Python 3.10.20** until the PI rules (next section).
4. **ArduPilot is pinned** to `Copter-4.5.7` =
   `2a3dc4b7bf2507120f7378a7b2fde73185e0c325` in `sitl/setup_sitl.sh`.
   `bash sitl/setup_sitl.sh --verify` checks a checkout against the pin and
   changes nothing. `~/ardupilot` fails that check today on one nested
   submodule (see "ArduPilot pin").

## What runs where (measured 2026-10-06)

| Environment | Python | Pins that matter | Runs |
|---|---|---|---|
| conda `vla-real` | **3.10.20** | torch 2.6.0+cu124, torchvision 0.21.0+cu124, transformers 4.40.1, tokenizers 0.19.1, timm 0.9.16, accelerate 0.32.1, bitsandbytes 0.49.2, peft 0.11.1, numpy 1.26.4, pydantic 2.13.4, shapely 2.1.2, jinja2 3.1.6, cryptography 49.0.0, projectairsim 0.2.0, airsim 1.8.1, opencv-contrib-python 5.0.0.93, Pillow 12.2.0, jsonschema 4.26.0 | Every Project AirSim / CityLife flight (`demo/follow_vlm.py` through `scripts/run_follow_vlm.ps1`, `run_citylife_follow.ps1`, and the other `scripts/*.ps1`). Every OpenVLA / AerialVLA flight (`demo/real_vla_demo.py`, `aerialvla_demo.py`, `aerialvla_pas_demo.py`). The test suite as `TUTORIAL.md:118` runs it. Bundle signing (it has cryptography). |
| conda `vla-drone` | 3.11.15 | pydantic 2.13.4, shapely 2.1.2, numpy 2.4.6, pyyaml 6.0.3, jinja2 3.1.6, torch 2.6.0+cu124, fastapi 0.139.0, uvicorn 0.50.2. **No** cryptography, opencv, Pillow, projectairsim, transformers, jsonschema | The env `requirements.txt:2` names; the unit tests in `RUNBOOK.md:100`; the hot-apply API (`guardrail/api.py`, fastapi + uvicorn). |
| conda `pas` | 3.11.15 | projectairsim 0.2.0, opencv-python 5.0.0.93, Pillow 12.3.0, cryptography 49.0.0, jsonschema 4.26.0, pydantic 2.13.4, shapely 2.1.2, numpy 2.4.6. **No** jinja2, torch, transformers | `tools/build_eval_data.py`; Project AirSim client samples. (`docs/projectairsim-setup.md:12` still says "py 3.10"; it is 3.11.15.) |
| conda `airsim` | 3.10.20 | airsim 1.8.1, msgpack-rpc-python 0.4.1, pydantic 2.13.4, shapely 2.1.2, numpy 2.2.6, jinja2 3.1.6, opencv-contrib-python 5.0.0.93, Pillow 12.3.0, torch 2.12.1+cpu, fastapi 0.139.0, uvicorn 0.50.2. **No** cryptography | The legacy AirSim 1.8 rail: `demo/run_demo.py` (`RUNBOOK.md` rail 1), `scripts/fly.ps1`, `scripts/demo_latest.ps1`. |
| WSL `~/venv-ros` (system site-packages) | 3.12.3 | ROS 2 Jazzy on Ubuntu 24.04.4; pydantic 2.13.4 and shapely 2.1.2 (in the venv); numpy 1.26.4, PyYAML 6.0.1, jinja2 3.1.2 and cryptography 41.0.7 (from `/usr/lib/python3/dist-packages`); pymavlink 2.4.49 (from `~/.local`) | The KPI rail: `sitl/ros2_shield_node.py` + `ros2_vla_stub_node.py` through `sitl/run_ros2_demo.sh`, which produced all five canonical KPI runs. |
| WSL `~/venv-ap` | 3.12.3 | empy 3.3.4, pexpect 4.9.0, future 1.0.0, pymavlink 2.4.49, MAVProxy 1.8.74, pydantic 2.13.4, shapely 2.1.2, PyYAML 6.0.3, numpy 2.5.0 (installed 2026-07-03, pulled in unpinned). **No** jinja2, cryptography | The ArduPilot build (waf) and the pymavlink rail, `sitl/run_sitl_demo.py`. Without jinja2 that rail now stops at `build_prompt()` (see "Open items"). |

The Shield therefore runs on **3.12.3** on the KPI rail and on **3.10.20** on
the Project AirSim rail, where `demo/follow_vlm.py` hosts the detector, the
simulator client, and the Shield in one process.

## Why vla-real stays on Python 3.10.20

The OpenVLA stack in `vla-real` was put together by trial on 2026-07-14..16.
It is the only combination on record that loads OpenVLA-7B in 4-bit and flies
on this Windows machine (project memory `real-vla-integration`). Each pin
there was forced by a failure:

- accelerate 0.32.1: 1.x breaks 4-bit dispatch with transformers 4.40.
- peft 0.11.1: 0.17 needs transformers newer than 4.40.
- timm 0.9.16: must stay below 1.0. The checkpoint's own remote code
  (`D:/models/openvla-7b/modeling_prismatic.py`) builds its vision backbone
  with `timm.create_model` and patches timm's `LayerScale`.
- transformers 4.40.1 / tokenizers 0.19.1: the checkpoint was saved with
  transformers 4.40.1 (`"transformers_version": "4.40.1"` in its config).
- numpy 1.26.4: opencv pulls in numpy 2 and has to be re-pinned after any
  `pip install`.

**None of these pins is itself a Python 3.10 requirement, as far as can be
checked offline.** torch 2.6.0+cu124 imports on 3.11.15 (`vla-drone`). `pas`
runs projectairsim, opencv, and Pillow on 3.11.15. The reason to stay is
validation cost, not incompatibility. Every OpenVLA / AerialVLA flight and all
Project AirSim / CityLife evidence since July were produced in this
environment. Seven of the nine `scripts/*.ps1` launchers point at it; `fly.ps1`
and `demo_latest.ps1` use the legacy `airsim` env. Rebuilding it on 3.11 means re-resolving the
stack above, then re-flying to show that nothing moved. This unit did not
attempt that (no package installs, no GPU model loads).

**What the deviation actually covers.** In `vla-real`, the Shield, the compiler,
and the KPI code run inside the flight process on 3.10.20. That code is
non-VLA, and the lock applies to it, so the deviation is real on the
perception rail. It does not touch the KPI rail, where the Shield runs on
3.12.3.

**Ways to close it:**

- (a) The PI accepts it in writing. This is recommended in
  `PI-DECISIONS-2026-10-06.md` Q4 item 2.
- (b) Rebuild `vla-real` on 3.11 and re-fly a reference flight.
- (c) Move the Shield out of the flight process onto 3.11+, as the ROS 2 rail
  already does.

Until one of these happens, rule 2 above, "importable on 3.10", is
load-bearing.

## Test results on Python 3.11 (2026-10-06)

Each `tests/test_*.py` file was run one at a time with
`C:/Users/natha/.conda/envs/vla-drone/python.exe -B tests/<file>`. Where a file
failed, it was re-run in `vla-real` (3.10.20, the baseline) and in `pas`
(3.11.15, which has the packages `vla-drone` lacks). Other units were editing
the tree during these runs, so this is a snapshot. The coordinator's full run
after all units finish supersedes it.

**vla-drone (3.11.15), run 12:08-12:12 (the latest of three):**

- 53 files; 48 exit 0, 5 exit 1.
- 1223 tests: 1175 pass, 34 fail, 14 skipped. Every skip is named, and no
  runner counts one as passed, except the one noted below.
- The 14 skips are all packages `vla-drone` does not have. Nine are the
  expected signing gap: 6 in `test_bundle` and 3 in `test_replay`, which say
  "needs cryptography". The other five need jsonschema (`test_policy_hash` 1,
  `test_csp` 1), tokenizers (`test_csp` 1, `test_real_vla_prompt` 1) or the
  `airsim` package (`test_real_vla_prompt` 1).
- `test_sweep`, which crashed at import (`KeyError: 'id'`, a scenario-format
  change in progress) in the 11:24 run, now passes 39/39. One of the 39,
  `test_the_sitl_backend_reports_unavailable_rather_than_pretending`, prints
  "SKIP: this host does have the ArduPilot rail" and is still counted as
  passed by that file's runner.

The earlier runs (finished 11:04 and 11:28: 44 and 49 files) show the same 34
failures. All 34 are `ModuleNotFoundError` for a package that `vla-drone` does
not have. Re-run at 12:15, each file passes in full both on 3.10.20 and on
3.11.15 (`pas`), so Python 3.11 is not the cause:

| File | vla-drone | Missing package | vla-real 3.10.20 | pas 3.11.15 |
|---|---|---|---|---|
| test_capture_pairing | 3/8 | cv2 (5) | 8/8 | 8/8 |
| test_city_traffic | 33/34 | projectairsim (1) | 34/34 | 34/34 |
| test_policy_hud | 30/36 | PIL (6) | 36/36 | 36/36 |
| test_range_and_lock | 85/101 | cv2 (16) | 101/101 | 101/101 |
| test_two_view | 3/9 | cv2 (5), PIL (1) | 9/9 | 9/9 |

The signing code also works on 3.11. In `pas`, which has cryptography, the
tests `vla-drone` skips all run and pass: `test_replay` 20/20,
`test_policy_hash` 20/20 (including the jsonschema test), and every signing
test in `test_bundle`. `test_bundle` is 37/40 there; the 3 failures are
`pas` lacking jinja2, and none is a signing test.

`test_city_traffic`'s one failure is a hidden dependency.
`test_update_stagger_skips_the_rpc_but_keeps_truth_current` drives
`demo/moving_car.py`, which imports `projectairsim.types` (line 391) to build
the teleport pose. Without the package, every teleport is caught and logged,
the fake world counts zero poses, and the test fails. It fails visibly, not
silently.

A full `pas` run that finished at 11:10 caught two files mid-edit by other
units: `test_csp` (1/41) and `test_replay` (6/15, a policy-hash scheme change
in progress). Both pass in the 12:08 `vla-drone` run: `test_csp` 41/43 with 2
skipped, and `test_replay` 17/20 with 3 skipped. `test_csp` in `pas` is 9/48
(12:15) only because `pas` has no jinja2.

**No test failed because of Python 3.11 itself.** Every failure or skip is
either a package missing from that environment (cv2, PIL, projectairsim,
cryptography, jsonschema, tokenizers, airsim; jinja2 in `pas`) or code that
was mid-edit at the time. Mid-edit failures fail the same way on 3.10.20.
Four tests have not passed on any 3.11 environment yet: the two that need
tokenizers and the one that needs the `airsim` package (no 3.11 env here has
those packages), and `test_csp`'s schema test, which needs jsonschema and
jinja2 together (`vla-drone` has only jinja2, `pas` only jsonschema).

## ArduPilot pin

| Period | `~/ardupilot` HEAD | Evidence |
|---|---|---|
| 2026-07-03 11:04 → 2026-09-18 09:03 | master `5f119834545431c19d0e69fb4db334bf604f1549` (`ArduPilot-4.6.0-beta1-7337-g5f11983454`; `ArduCopter/version.h`: "ArduCopter V4.8.0-dev"), unreleased | reflog `clone: from https://github.com/ArduPilot/ardupilot.git`. All 56 dataflash logs in `~/sitl-run/logs` (2026-07-03..08-31) carry the banner "ArduCopter V4.8.0-dev (5f119834)". `~/sitl-run/mavros.log` (2026-08-31 19:31, the `ros2_ped_*` runs) has "VER: 1.1: Flight software: 04080000 (5f119834)" (4.8.0, type dev). 4 more logs in `~/ardupilot/logs` (09-17 22:50..09-18 07:00) show the same banner. |
| 2026-09-18 09:03 → today | `Copter-4.5.7` = `2a3dc4b7bf2507120f7378a7b2fde73185e0c325` (commit 2024-10-15, "Plane: version to 4.5.7") | reflog `checkout: moving from master to Copter-4.5.7`. `build/sitl/ap_version.h` GIT_VERSION "2a3dc4b7" (09-18 09:03:24). `build/sitl/bin/arducopter` built 09-18 09:05, banner "ArduCopter V4.5.7 (2a3dc4b7)". 23 logs in `~/ardupilot/logs` (09-18 13:03..09-23 14:56) carry that banner. |

**Consequence.** All twelve SITL runs in `demo/out` are dated 2026-08-24..31:
seven `sitl_*` on the pymavlink rail and the five canonical `ros2_*` KPI runs.
They flew on ArduCopter **V4.8.0-dev (5f119834)**, an unreleased master build,
not on the pinned release. That leaves two ways forward. One is to re-fly the
canonical runs on the pinned firmware (the re-fly that ARCH-19/22/29 already
plan). The other is to rebuild the August firmware exactly with
`ARDUPILOT_REF=5f119834545431c19d0e69fb4db334bf604f1549 ARDUPILOT_SHA=5f119834545431c19d0e69fb4db334bf604f1549 bash sitl/setup_sitl.sh`.
Until one of these is done, any report quoting the SITL KPIs should name
4.8.0-dev (5f119834) as the autopilot.

Nothing records why the tree moved to 4.5.7 on 18 September: no commit,
WORKLOG entry, or memory note mentions it. It is pinned because it is what has
been built and flown since then, and because a release tag is something real
firmware can match. Master at a random date is not.

**What the script now does.**

- It clones without `--recursive`. Cloning master's submodules and then
  switching to the pin is how `~/ardupilot` came to carry an untracked
  `modules/littlefs`.
- It detach-checks-out `ARDUPILOT_REF` and refuses to build unless
  `HEAD == ARDUPILOT_SHA`.
- It sets every submodule recursively to the pin.
- It pins the `~/venv-ap` packages to the versions measured today, and every
  core dependency of the guardrail package among them. That adds
  `jinja2==3.1.6`, because `run_sitl_demo.py` calls `build_prompt()` to write
  `prompt.yaml`, and that now renders Jinja2 templates. It also adds `numpy==2.5.0`, which
  was already installed but had arrived unpinned.
- It sets `pipefail`, so `./waf ... | tail` can no longer hide a failed build.
- It ends in the same check that `--verify` runs: HEAD, submodules, tracked
  files (inside submodules too), and the version banner baked into the binary.
- The tracked-file check uses `git status --ignore-submodules=untracked`. An
  edited `pymavlink/gen.py` at the right commit generates different headers,
  just as a wrong commit does, so edits inside any submodule fail the check.
  Submodules whose only change is untracked files (crash dumps) are skipped.
  The earlier `--ignore-submodules=all` passed such an edit as PIN OK; the
  2026-10-06 review found this.
- A git call that fails, fails the check. `verify_pin` runs under
  `if verify_pin`, where bash suspends `set -e`. There, a crashed
  `git status`, or a `git submodule status` that exits 128 (a gitlink with no
  `.gitmodules` entry), printed nothing, and nothing read as "clean": PIN OK.
  Both exit statuses are now checked.
- `--verify` writes nothing. Its git calls run with `--no-optional-locks`, and
  git passes that on to the `git status` runs it spawns inside submodules
  (`GIT_OPTIONAL_LOCKS=0`). It does not use `git describe --dirty`. That flag
  refreshes and rewrites `.git/index` even under `--no-optional-locks`
  (measured on 2026-10-06 with git 2.53 on a scratch repository with a stale
  stat cache; `git status` honours the flag). The first version of the check
  used `--dirty`, so its one run against `~/ardupilot` earlier that day may
  have refreshed that index's stat cache. That changes no file content and no
  commit.

**The current tree fails that check.** `bash sitl/setup_sitl.sh --verify`
(final version of the check, run in WSL on 2026-10-06 after the review fixes)
printed, and exited 1:

```
pin      : Copter-4.5.7 = 2a3dc4b7bf2507120f7378a7b2fde73185e0c325
checkout : 2a3dc4b7bf2507120f7378a7b2fde73185e0c325 (Copter-4.5.7)
PIN FAIL: submodules differ from what the checked-out commit records:
+ec06837a50688b5974094876c3c9d1b8bf86629d modules/mavlink/pymavlink (v2.4.49-51-gec06837a)
PIN FAIL: tracked ArduPilot files modified:
 M modules/mavlink
binary   : ArduCopter V4.5.7 (2a3dc4b7)
```

All 22 git indexes in `~/ardupilot` (the superproject and 21 submodules)
hashed byte-identical before and after that run (WSL git 2.43.0). The
` M modules/mavlink` line is the same pymavlink drift, seen one level up. No
other submodule has an edited tracked file.

HEAD and the binary match the pin. The nested `pymavlink` does not. It is
still at `ec06837a` (2026-02-27), left over from the master clone, while
`Copter-4.5.7`'s `modules/mavlink` records `8ba67079`. pymavlink's generator
writes the MAVLink C headers the firmware is compiled against, so the binary
on disk is 4.5.7 source built against headers from a newer generator. Whether
its bytes differ from a clean 4.5.7 build was not measured. The fix is to
re-run `bash sitl/setup_sitl.sh` in WSL: it resets the submodule, rebuilds
(about 15 minutes), and ends in the check. This unit could not do that, because
it was not allowed to build. Untracked crash dumps (`dumpcore.sh_*.out`,
`dumpstack.sh_*.out`) and `modules/littlefs` are ignored by the check.

The same final check with the August pin (`ARDUPILOT_REF`/`ARDUPILOT_SHA` set
to 5f119834) fails four ways: HEAD, submodules, the same ` M modules/mavlink`,
and a binary not built from 5f119834. The indexes were again unchanged.

`tests/test_env_pins.py` runs the whole script offline against throwaway
repositories. They hold an upstream stand-in (a release tag, a newer master,
and a nested `modules/mavlink/pymavlink` pair), a stub `~/venv-ap`, and a fake
`waf` that writes HEAD's 8-hex banner into the binary. The script must:

- build the pin from a fresh clone of the newer master, with every submodule
  level at the release's commits, and re-run cleanly;
- reach the pin from a checkout that lacks the tag, by fetching tags;
- refuse, before building, a tag that resolves to another commit;
- fail when the binary is not the pin;
- fail when a rebuild fails, even with a good old binary still on disk;
- refuse, rather than discard, local edits, with git's own reason instead of
  a "moved tag" message;
- install every core dependency of `pyproject.toml` pinned in `~/venv-ap`;
- and in `--verify`, refuse a wrong commit, a wrong banner, a binary with no
  banner, an edited tracked file, an edited tracked file inside the nested
  `pymavlink` or in `modules/mavlink`, a nested submodule at another commit
  (the state of the real tree), a missing binary, and a missing checkout. It
  must also refuse when `git status` itself fails, or when
  `git submodule status` fails (a gitlink with no `.gitmodules` entry). It
  must not count untracked crash dumps, at the top level or inside a
  submodule, as edits. And it must leave all three indexes (superproject and
  both submodules) byte-identical, with stale stat caches at every level.

The fixtures cannot reach the network even if the script regresses. The
`--verify` runs point `ARDUPILOT_URL` at a path that does not exist, and every
fixture home has a stub `~/venv-ap`. So a broken `--verify` dispatch that fell
through to the full setup fails offline at once. The bash used is accepted
only if, handed a fresh `HOME`, it reads back a token from a file in that
directory. Any WSL launcher fails that probe, whatever its name or PATH
position, so the tests can never act on WSL's real `~/ardupilot`.

**How hard the tests bite (2026-10-06, 34 tests).** Thirty hand-made mutants
of the final script were each run against the test file. 28 fail at least one
test.

- 11 from the first round: dropping the submodule check, `exit 1` → `:`,
  dropping the final check, dropping `--no-optional-locks`,
  `checkout || true`, `--verify` always exiting 0, a non-recursive submodule
  update, `pipefail` turned into an `echo`, ignoring a wrong banner, blinding
  the dirty check, and putting `describe --dirty` back.
- 13 from the review: among them, accepting an empty banner, never fetching
  tags, and dropping jinja2 from `~/venv-ap`. Those three passed the previous
  version of the test file.
- 6 that undo this round's fixes, one each: `--ignore-submodules=all` back,
  `=none` (over-strict), the `git submodule status` exit code swallowed or
  not counted, the `git status` exit code unchecked, and numpy dropped from
  `~/venv-ap`.

The two survivors are removing `git submodule sync` and testing the binary
with `-e` instead of `-x`. Neither changes anything in these fixtures. The
fixture's submodule URLs never change between master and the pin, so `sync`
has nothing to do. The fake binaries are always executable. They are kept
in the script for real trees, not proven by a test.

On the code as reviewed (before these fixes), the same file passes 30/34. The
four failures are the four tests aimed at the fixes: an edit inside a
submodule, a failing `git status`, a failing `git submodule status`, and an
unpinned core dependency in `~/venv-ap`. On the script from commit 1d09786
with no `pyproject.toml`, it passes 3/34. Those three are the import, 3.10
syntax and line-ending checks, which neither change touches.

**Recording it per run** is not done yet: the field belongs in the run record.
MAVROS already logs the autopilot's own `AUTOPILOT_VERSION` ("Flight software:
04080000 (5f119834)"). That reading is the version that actually flew, which
is better evidence than a script default. It should go into `metrics.json`
`hil_evidence.ardupilot_version`, and `check_hil_evidence()` should refuse a
`dev` run without it.

That refusal must land together with a retro-fill. None of the five stored
canonical `ros2_*` runs records the field. Made mandatory on its own, the
check would demote all five the next time `tools/rescore_kpis.py` runs, and
the demotion would look like a regression, not a missing label. Their
manifests should first get `"ardupilot_version": "ArduCopter V4.8.0-dev
(5f119834)"`, taken from the evidence measured above: the MAVROS line
"Flight software: 04080000 (5f119834)" in `~/sitl-run/mavros.log`, and the
banner carried by all 56 dataflash logs in `~/sitl-run/logs`. After the
retro-fill, a rescore should leave all five as they were; any run that still
lacks the field is then correctly refused.

## ROS 2 distribution

The rail uses **Jazzy** on Ubuntu 24.04.4 (`/opt/ros/jazzy`). The grant lists
the distribution as an open question and names Humble for AI Wings parity. The
reference implementation uses Humble. No PI decision is recorded. The question
is `PI-DECISIONS-2026-10-06.md` Q4 item 3 ("confirm Jazzy") and is not decided
here.

## Open items (owned by other files)

- `README.md` still has the sentence "Requires ProjectAirSim with an Unreal
  city level, an NVIDIA GPU, and Python 3.10." It should state the split: the
  guardrail package needs Python 3.11+, and the OpenVLA / Project AirSim
  flight env (`vla-real`) is 3.10.20, a recorded deviation.
- `requirements.txt` lacks `jinja2`. `guardrail/compiler.py` needs it for
  `build_prompt()` and `summary_pack()`. `pyproject.toml` lists it, so the two
  files disagree until `requirements.txt` gains the line, and
  `test_core_dependencies_are_exactly_requirements_txt` fails until then. With
  the line added, the file passes in full (checked with a scratch copy of
  `requirements.txt` through `ENV_PINS_REQUIREMENTS`).
- **The pymavlink rail is broken in `~/venv-ap` until jinja2 is installed.**
  `sitl/run_sitl_demo.py` calls `compiler.build_prompt()` to write
  `prompt.yaml` before take-off. That call now imports jinja2, and `~/venv-ap`
  has none (re-read 2026-10-06). With jinja2 blocked, `parse_command()` still
  works and `build_prompt()` raises `ImportError` ("install jinja2"). The KPI
  rail (`~/venv-ros`, jinja2 3.1.2 from the system) is not affected. Re-running
  `setup_sitl.sh` installs it, but also triggers the 15-minute rebuild. The fix
  that needs no rebuild is one user command in WSL:
  `~/venv-ap/bin/pip install jinja2==3.1.6`.
- `pas` lacks jinja2, so `tools/build_eval_data.py`, which runs the suite in
  `pas`, will count every test that renders rule text as failed (on
  2026-10-06: `test_csp` 9/48, `test_real_vla_prompt` 13/33, `test_bundle`
  37/40 there).
- `docs/projectairsim-setup.md:12` says `pas` is py 3.10; it is 3.11.15.
- The `guardrail/bundle.py` docstring says the WSL flight venvs lack
  cryptography. `~/venv-ros` has 41.0.7 from the system site-packages; only
  `~/venv-ap` lacks it.
- The per-run ArduPilot version field, with the retro-fill of the five
  canonical `ros2_*` manifests that must come with it (previous section).

## How to check

```bash
python tests/test_env_pins.py                    # pyproject vs requirements/VERSION/imports; the pin; setup_sitl.sh run offline on fixtures (about 60 s)
wsl bash "sitl/setup_sitl.sh" --verify           # read-only: is ~/ardupilot the pinned firmware?
```

The floor is enforced, not just written. A wheel built offline from
`pyproject.toml` with the 3.11.15 env (`pip wheel --no-deps
--no-build-isolation --no-index`, in a scratch copy that also held decoy
`demo/`, `tools/` and `training/` packages) contains only `guardrail/` (41
files: 21 modules and all 20 `.j2` templates) and its `.dist-info`. Its
metadata says `Requires-Python: >=3.11`, lists the five core dependencies,
and lists the extras `api`, `signing` and `tokens`. On 3.10.20, pip refuses
both to build it and to install it ("requires a different Python: 3.10.20 not
in '>=3.11'"). A `pip install --dry-run` on 3.11.15 accepts it. All of this
was rebuilt and re-checked on 2026-10-06 after the review fixes.
