# Launcher scripts

Consolidated here from the repo root on 2026-09-16 (`git mv`, history preserved).
Nothing in this folder was rewritten in behaviour — the three that derive their
own location (`run_follow_vlm.ps1`, `run_follow_car.ps1`, `run_retarget_demo.ps1`)
had their root-detection line adjusted for the one extra directory level; every
other script here already hardcoded the repo root as an absolute path and needed
no change.

**Found while moving these, worth stating plainly rather than leaving implicit:**
this folder mixes launchers for two different simulator backends. Neither list
below is a judgment that one is dead — that decision is not this cleanup's to
make — it is what each script's own contents say it talks to.

## Current backend — Project AirSim / Unreal, port 8989, `PASBlocks/Blocks.uproject`

- `run_follow_vlm.ps1` — the tracking demo. Linked from `TUTORIAL.md` at the repo root.
- `run_retarget_demo.ps1` — the class-conditional stand-off demo (phrase changes
  target class mid-flight; see `docs/FINDING-the-standoff-rule-that-never-armed.md`
  and the retarget flights under `demo/out/`).
- `run_follow_car.ps1` — an earlier "follow the car" experiment; its own docstring
  records a negative result (no direction hint → the drone never moves) as the
  finding, not a fault.
- `demo_compare_guardrail.ps1`, `demo_japanesecity.ps1` (+ its double-click wrapper
  `run_japanesecity.bat`) — guardrail on/off and one-click JapaneseCity demos.

## Older backend — standalone AirSim `.exe` worlds, port 41451

- `fly.ps1` (+ `fly.bat`) — launches `D:\AirSim\<World>\WindowsNoEditor\<World>.exe`
  directly and drives `demo\run_demo.py`. `RUNBOOK.md` currently calls this the
  "fastest way" to fly; that line was written before the move to Project AirSim
  and was not re-verified as part of this cleanup.
- `demo_latest.ps1`, `demo_real_vla.ps1` — same standalone-AirSim path.

If the project has fully moved off the standalone AirSim binaries, the second
group (and the `RUNBOOK.md` line pointing at `fly.ps1`) is worth a deliberate
look — but that is a "does this still work" question, not a filing question,
and this cleanup only answers the filing question.
