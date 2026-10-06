#!/usr/bin/env bash
# ArduPilot SITL setup for WSL Ubuntu 24.04 — run as user nathan.
# Idempotent: safe to re-run; skips finished steps.
#
#   bash setup_sitl.sh            # clone/checkout the pin, venv, build, verify
#   bash setup_sitl.sh --verify   # read-only: does ~/ardupilot match the pin?
#
# ArduPilot is PINNED, not "whatever master was that day" (ARCH-31). The grant
# gives the ArduPilot inner loop as "Bit-identical between SITL and firmware"
# (Architecture constraints p3) and wants every quoted number traceable to the
# code that produced it (six-field manifest, Stress Testing p1). Neither means
# anything if a fresh setup builds a different autopilot from the one that
# flew, which is what the unpinned `git clone --recursive` of master did.
#
# The pin is the tree this PC has built and flown since 2026-09-18 09:03, read
# on 2026-10-06 with `git -C ~/ardupilot describe --tags --always` and
# `git rev-parse HEAD`: Copter-4.5.7 = 2a3dc4b7bf25..., whose binary reports
# "ArduCopter V4.5.7 (2a3dc4b7)".
#
# It is NOT the firmware behind the stored SITL runs. ~/ardupilot was cloned
# from master on 2026-07-03 at 5f119834545431c19d0e69fb4db334bf604f1549, and
# every dataflash log written before the 2026-09-18 checkout (~/sitl-run/logs,
# 2026-07-03..08-31, covering all twelve demo/out/sitl_* and ros2_* runs)
# reports "ArduCopter V4.8.0-dev (5f119834)", an unreleased master build. To
# rebuild exactly that firmware, override both values:
#   ARDUPILOT_REF=5f119834545431c19d0e69fb4db334bf604f1549 \
#   ARDUPILOT_SHA=5f119834545431c19d0e69fb4db334bf604f1549 bash setup_sitl.sh
# docs/DESIGN-python-versions.md ("ArduPilot pin") has the evidence.
set -eo pipefail   # pipefail: `./waf ... | tail` must not hide a failed build

ARDUPILOT_URL="${ARDUPILOT_URL:-https://github.com/ArduPilot/ardupilot.git}"
ARDUPILOT_REF="${ARDUPILOT_REF:-Copter-4.5.7}"
ARDUPILOT_SHA="${ARDUPILOT_SHA:-2a3dc4b7bf2507120f7378a7b2fde73185e0c325}"
AP_DIR="$HOME/ardupilot"

# Read-only check that the checkout, its submodules and the built binary are
# all the pinned firmware. Prints what it found either way, so a run record
# can quote the measured banner instead of trusting this script's defaults.
# --no-optional-locks keeps `git status` from rewriting the index; git passes
# it on as GIT_OPTIONAL_LOCKS=0, so the submodule `git status` runs that the
# dirty check spawns write nothing either (all 22 indexes of ~/ardupilot
# hashed unchanged, 2026-10-06, git 2.43.0).
#
# Every git call whose output is read as "nothing wrong" has its exit status
# checked. This function runs under `if verify_pin`, where bash suspends
# `set -e`, so a crashed `git status` would otherwise print nothing, and
# nothing reads as clean ("silence reads as success").
verify_pin() {
  local fail=0 head all_subs subs dirty bin banner
  local git_ro=(git --no-optional-locks -C "$AP_DIR")
  if [ ! -d "$AP_DIR/.git" ]; then
    echo "PIN FAIL: $AP_DIR is not a git checkout (run setup_sitl.sh)"
    return 1
  fi
  head="$("${git_ro[@]}" rev-parse HEAD)"
  echo "pin      : $ARDUPILOT_REF = $ARDUPILOT_SHA"
  # No `describe --dirty`: it refreshes and rewrites .git/index even under
  # --no-optional-locks (measured 2026-10-06, git 2.53), which would make this
  # "read-only" check write to the tree it inspects. The dirty-tree check below
  # uses `git status`, which honours the flag.
  echo "checkout : $head ($("${git_ro[@]}" describe --tags --always))"
  if [ "$head" != "$ARDUPILOT_SHA" ]; then
    echo "PIN FAIL: HEAD is not the pinned commit"
    fail=1
  fi
  # '+' = submodule at a different commit than the checked-out commit records,
  # '-' = not initialised, 'U' = conflict. The nested modules/mavlink/pymavlink counts:
  # its generator writes the MAVLink headers the firmware is compiled against,
  # so a stale pymavlink gives a binary that is not the pinned firmware even
  # with HEAD right.
  if ! all_subs="$("${git_ro[@]}" submodule status --recursive)"; then
    # e.g. a gitlink with no .gitmodules entry: git exits 128 and lists
    # nothing, which `| grep || true` used to turn into "no bad submodules".
    echo "PIN FAIL: git submodule status failed; cannot tell what the submodules are"
    fail=1
  fi
  subs="$(printf '%s\n' "$all_subs" | grep -E '^[-+U]' || true)"
  if [ -n "$subs" ]; then
    echo "PIN FAIL: submodules differ from what the checked-out commit records:"
    echo "$subs"
    fail=1
  fi
  # Tracked files edited anywhere, submodules included: an edited
  # pymavlink/gen.py at the right commit generates different headers just as
  # a wrong commit does. =untracked (not =all) looks inside submodules but
  # skips submodules whose only change is untracked files; --untracked-files=no
  # does the same for the top-level tree (crash dumps, the master-only
  # modules/littlefs). On ~/ardupilot today it adds one line,
  # " M modules/mavlink", which is the pymavlink drift seen from one level up.
  if ! dirty="$("${git_ro[@]}" status --porcelain --untracked-files=no --ignore-submodules=untracked)"; then
    echo "PIN FAIL: git status failed; cannot tell whether tracked files were edited"
    fail=1
  fi
  if [ -n "$dirty" ]; then
    echo "PIN FAIL: tracked ArduPilot files modified:"
    echo "$dirty"
    fail=1
  fi
  bin="$AP_DIR/build/sitl/bin/arducopter"
  if [ ! -x "$bin" ]; then
    echo "PIN FAIL: $bin missing (not built)"
    fail=1
  else
    # waf bakes the first 8 hex digits of HEAD into the version banner
    # (build/sitl/ap_version.h GIT_VERSION); it is also what the autopilot
    # announces over MAVLink at boot. Limit: it proves which commit was
    # built, not which submodule commits were checked out at build time.
    banner="$(grep -a -o -m1 -E 'ArduCopter V[0-9][^ ]* \([0-9a-f]{8}\)' "$bin" | head -n1 || true)"
    echo "binary   : ${banner:-<no version banner found>}"
    case "$banner" in
      *"(${ARDUPILOT_SHA:0:8})") ;;
      *) echo "PIN FAIL: binary was not built from the pinned commit"; fail=1 ;;
    esac
  fi
  if [ "$fail" -eq 0 ]; then echo "PIN OK"; fi
  return "$fail"
}

if [ "${1:-}" = "--verify" ]; then
  if verify_pin; then exit 0; else exit 1; fi
fi

echo "=== [1/5] ardupilot at the pin ($ARDUPILOT_REF) ==="
cd ~
if [ ! -d ardupilot ]; then
  # No --recursive: cloning master's submodules and then switching to the pin
  # leaves master-only submodule directories behind (how ~/ardupilot came to
  # carry an untracked modules/littlefs). Submodules come below, at the
  # commits the pin records.
  git clone "$ARDUPILOT_URL" ardupilot
else
  echo "already cloned"
fi
cd "$AP_DIR"
if [ "$(git rev-parse HEAD)" != "$ARDUPILOT_SHA" ]; then
  git rev-parse -q --verify "$ARDUPILOT_REF^{commit}" > /dev/null \
    || git fetch --tags origin
  # Refuses (and set -e stops here) if local edits would be overwritten;
  # it never discards work.
  git -c advice.detachedHead=false checkout --detach "$ARDUPILOT_REF"
fi
actual="$(git rev-parse HEAD)"
if [ "$actual" != "$ARDUPILOT_SHA" ]; then
  echo "PIN FAIL: $ARDUPILOT_REF resolved to $actual, expected $ARDUPILOT_SHA"
  echo "(moved tag or different remote; refusing to build an unpinned tree)"
  exit 1
fi
# --recursive matters: see verify_pin on modules/mavlink/pymavlink.
git submodule sync --recursive > /dev/null
git submodule update --init --recursive
echo "at $(git describe --tags --always) = $actual"

echo "=== [2/5] python venv + deps ==="
if [ ! -d ~/venv-ap ]; then
  python3 -m venv ~/venv-ap
fi
~/venv-ap/bin/pip install -q -U pip
# Versions as installed in ~/venv-ap on 2026-10-06 (Python 3.12.3), the venv
# behind sitl/run_sitl_demo.py. pymavlink is the rail's MAVLink path, and
# empy must stay 3.x for ArduPilot's code generators. The last five are the
# guardrail package's core dependencies (pyproject.toml), each pinned, so the
# Shield in this venv runs on named versions; numpy 2.5.0 was already there,
# pulled in unpinned. jinja2 is new: the compiler renders its rule text from
# guardrail/templates/*.j2 and run_sitl_demo.py calls build_prompt() to write
# prompt.yaml, so the existing ~/venv-ap, which has no jinja2, cannot run that rail
# until this step re-runs, or until `~/venv-ap/bin/pip install jinja2==3.1.6`
# (no rebuild needed). 3.1.6 is what vla-real and vla-drone run.
~/venv-ap/bin/pip install -q "empy==3.3.4" "pexpect==4.9.0" "future==1.0.0" \
  "pymavlink==2.4.49" "MAVProxy==1.8.74" \
  "pydantic==2.13.4" "pyyaml==6.0.3" "shapely==2.1.2" "numpy==2.5.0" "jinja2==3.1.6"
echo "venv ready"

echo "=== [3/5] waf configure ==="
source ~/venv-ap/bin/activate
./waf configure --board sitl 2>&1 | tail -3

echo "=== [4/5] build copter (this is the long part) ==="
./waf copter -j"$(nproc)" 2>&1 | tail -5

echo "=== [5/5] verify the build is the pinned firmware ==="
ls -la build/sitl/bin/arducopter
verify_pin
echo "=== DONE ==="
