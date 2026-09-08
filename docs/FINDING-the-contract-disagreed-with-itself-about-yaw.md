# The action contract disagreed with itself about yaw, and I made it worse first

**Date:** 2026-09-07
**Found by:** an adversarial review of the same day's work, after I had already
written the defect into a new module, a test, and a checklist entry marked CLOSED.
**Status:** fixed in four files, pinned by tests that ask the enforcing code
rather than the comment. **No stored KPI figure changes** — see the last section.

## The item this started from

`docs/CHECKLIST-remaining-work.md` item 9, open for weeks:

> Ours is world-frame (`vx` North, `vy` East); `vlaguard_common.Action4D` is
> body-frame. *"Both cannot be right, and no test compares them."*

I built `guardrail/frames.py` to close it, and reported that the frame mismatch
was benign but that I had found a second, sharper one the item never mentioned:

> `yaw_rate` is **deg/s** here and **rad/s** there — a factor of 57.3. A 45 deg/s
> cap misread as rad/s is 2578 deg/s, a limit nothing would ever bind against.

That was wrong, and the way it was wrong is the point of this document.

## What is actually true

There is **no unit mismatch with the reference**. Both sides are rad/s. The
mismatch was *inside our own package*, between a comment and the code that
enforces it:

| site | reading | |
|---|---|---|
| `guardrail/models.py:44` | `# deg/s` | **comment, wrong** |
| `guardrail/shield.py:467` | `# Action4D contract carries yaw_rate in RADIANS per second` | enforcing |
| `guardrail/shield.py:472,760` | `ymax = math.radians(k.yaw_rate_max_dps)` | enforcing |
| `guardrail/shield.py:536` | `yaw_dps = math.degrees(abs(action.yaw_rate))` | enforcing |
| `demo/follow_vlm.py` servo | `yaw_gain * bearing` — bearing in radians | the only live producer |
| `sitl/run_sitl_demo.py:145` | `math.radians(yaw_rate_dps)` | **adapter, wrong** |
| `sitl/ros2_shield_node.py:376` | `-math.radians(emitted.yaw_rate)  # CW dps -> CCW rad/s` | **adapter, wrong** |

Six sites read radians; one comment and two adapters read degrees. The comment
was written once and believed twice.

**Corrected 2026-09-08 — the table above missed a seventh site, and it was the
only one that published a number.** `guardrail/kpi.py::_repair_magnitude`
returned the raw rad/s yaw difference and `compute()` emitted it as
**`mean_yaw_repair_dps`**, a name that says degrees. Eight runs had stored a
figure 57.296× too small; all eight are corrected on disk (`city_kpi` 0.0536 →
3.0710, and so on). The survey missed it because it was built from a grep of the
sites that ENFORCE the cap, and this one only reports it — a partial survey
presented as an exhaustive one, which is the same habit as the run-count below.

**And the producer is not clipped.** The row above used to read
`np.clip(yaw_gain * bearing, -1.1, 1.1)`. That clip is in the lost-lock COAST
branch only; both live tracking paths are unclipped, and `retarget_demo`
recorded **2.375 rad/s = 136.1 °/s**.

Measured, with a Shield whose only rule is a 45 dps yaw cap:

```
a = from_body(2.0, 0.0, 0.0, 0.5, 0.0)     # 0.5 rad/s = 28.6 deg/s, legal
VIOLATION: kin-caps |yaw_rate| 1641.4 dps > max 45.0
REPAIR   : YawClamp yaw 1641.4 -> 45.0 dps
```

The Shield read 28.65 as 28.65 **rad/s**. My "safe boundary" module manufactured
a P1 violation out of a lawful turn.

## The adapters, and why nothing has burned

Both SITL adapters applied `math.radians()` to a value that was already radians,
dividing every commanded yaw by 57.3. On the canonical rail a 45 dps clamp would
have reached the autopilot as **0.785 dps**.

It never happened. The stub pilot that flies that rail has never commanded a
non-zero yaw rate — checked across **all twelve** runs whose manifest topology is
`ardupilot-sitl-pymavlink` or `canonical-hil`:

| topology | runs | ticks | max \|raw\| | max \|emitted\| |
|---|---|---|---|---|
| `ardupilot-sitl-pymavlink` | 7 | 1 922 | 0.0000 | 0.0000 |
| `canonical-hil` | 5 | 928 | 0.0000 | 0.0000 |
| **total** | **12** | **2 850** | **0.0000** | **0.0000** |

`0 / 57.3 = 0`. **Every stored KPI figure is unaffected**, and the fix changes no
published number. The defect was latent, waiting for the first flight that turned.

> **Corrected 2026-09-08.** This section first named three runs and called them
> "all three KPI-grade runs". Both halves were wrong: there are twelve runs on
> those topologies, and of the three named only `ros2_shield_on` is actually
> `kpi_grade` — `sitl_shield_on` and `sitl_ped_on` both record `kpi_grade: false`
> because `ardupilot-sitl-pymavlink` is not the grant's canonical topology. The
> five KPI-grade runs are all `ros2_*`. The conclusion is unchanged and now rests
> on 2 850 ticks instead of 562.

## One more inconsistency, deliberately left alone

`shield.py:423` forecasts heading as `yaw_deg=state.yaw_deg + action.yaw_rate * t`
— degrees plus radians-per-second. It is inert: `yaw_deg` is written by the
forecast and **read by no rule** (verified by grep across `guardrail/`). Changing
it would alter no decision, so it is recorded here rather than "fixed" in a way
nothing could test.

## The fix

- `guardrail/models.py` — the comment now says rad/s and says why, naming this
  document. The comment was the root cause; it is the first thing corrected.
- `sitl/run_sitl_demo.py` — parameter renamed `yaw_rate_rad_s`, conversion
  removed. MAVLink's `SET_POSITION_TARGET_LOCAL_NED` yaw_rate field is rad/s, so
  pass-through is correct rather than merely simpler.
- `sitl/ros2_shield_node.py` — a sign flip and nothing else.
- `guardrail/frames.py` — no longer converts, and its docstring now records that
  the conversion which looked like the careful thing to do *was* the bug.

`tests/test_frame_contract.py` was rewritten around one rule: **ask the code that
enforces the contract, never a comment.** It builds a Shield with a single yaw
cap and asserts that an action at the cap in the declared unit passes and one
over it does not; it reproduces the 1641 dps absurdity as a regression; and it
greps both adapters for a second conversion, since that defect was one call on
one line.

## Why I did not catch it and the review did

I read `models.py`, which is where the contract is declared, and did not read
`shield.py`, which is where it is enforced. Then I wrote a test that asserted my
reading back to me, and it passed — beside an existing test,
`test_yaw_cap_compares_degrees_with_degrees`, that has been pinning the opposite
convention green since 17 August. Two suites, two conventions, both green.

This is the same shape as the three defects found earlier the same week: the
artefacts were complete, consistent and wrong. The difference is that this one
was **created** by the work rather than uncovered by it, which makes it the more
useful entry — a checklist item marked `[CLOSED]` on the strength of a test that
tested a belief.

The rule that comes out of it: *a test that asserts a comment is not a test.*
Pin a contract to the code that enforces it, or to the wire format that consumes
it, and to nothing else.
