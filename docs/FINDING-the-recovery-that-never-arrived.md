# The recovery that never arrived

**Date:** 2026-09-01
**Branch:** `contract-gaps`
**Status:** found, fixed, and pinned by a regression test and a sweep scenario.

## What happened

Two of the grant's five acceptance KPIs — `mean repair magnitude` and
`mean time to safe` — had never been computed. Adding them meant deciding what
they mean, and the second definition turned out to be the whole exercise.

Measured the obvious way, off the `violations` already in every flight log,
`mean time to safe` reported **21.9 s** for `ros2_shield_on`. That run's
independently computed `nfz_s` and `alt_violation_s` are both **0.0**.

Both numbers were right. They were answering different questions.

## A violation means two different things

A tick carries a violation when either:

- the requested **action** is illegal — too fast, climbing too hard, aimed at a
  fence it has not reached yet; or
- the current **position** is illegal — already inside the zone, already below
  the floor.

`ros2_shield_on` logs 219 present-tense violations and **zero** illegal
positions. It flew alongside a no-fly zone for twenty seconds while the Shield
trimmed a pilot that kept turning into it. That is the Shield working exactly as
designed. Reporting it as *"it took 21.9 seconds to become safe"* would have
been false in a direction that flatters nobody.

This project has made that mistake once already. `det_hit_rate` counted
inferences that returned any box and was quoted for a week as evidence the
tracker held the right vehicle — see
`FINDING-the-hit-rate-was-not-a-hit-rate.md`. The fix is the same both times:
measure the thing the name promises.

## The test that settles it

`Shield.state_is_unsafe(state)` asks **"would standing still here be illegal?"**
If stopping is not allowed at this point, no choice of action makes the tick
safe, and the position itself is the problem.

`filter()` already relied on the idea — it re-checks `BRAKE` before daring to
brake, because a standstill inside a clearance ring is not a fail-safe. This just
gives the question a name so the KPI layer can record it per tick.

It cross-checks against numbers produced by an unrelated code path:

| run | unsafe ticks | independently logged dwell |
|---|---|---|
| `sitl_ped_on` | 216 | `alt_violation_s` 21.6 s |
| `ros2_shield_off` | 41 | `nfz_s` 3.7 s |
| `ros2_shield_on` | 0 | 0.0 s |

216 ticks at 0.1 s is 21.6 s exactly.

## What the corrected metric then found, immediately

The first scenario sweep that ran included a vehicle starting **below** the
altitude floor. `AltitudeFix` sizes the climb so the lookahead endpoint lands on
the floor:

```python
new = (env.alt_min_m - state.up) / self.lookahead_s
```

Aiming at the boundary makes the recovery a decaying exponential that converges
on it without ever crossing. Measured, from 3 m against a 10 m floor:

| time | altitude |
|---|---|
| 6 s | 9.10 m |
| 12 s | 9.88 m |
| 30 s | **9.99973 m** |

It never gets in. Not slowly — never, for any length of flight.

**Every existing KPI reported this as healthy, and each was correct to.** The
emitted action climbs, so it is legal; `p0_violation_escape_rate` stayed 0 the
whole way down; the repair count was high, which reads as the Shield working
hard. The action was always fine. The state never became safe, and nothing in
the KPI set could express that until `mean time to safe` existed.

The same bug had been faithfully copied into the corridor's altitude band, which
I wrote a day earlier.

## The fix

Recoveries aim a margin **inside** the band; a vehicle already inside still aims
at the boundary. The asymmetry is the point: a vehicle inside its envelope
flying level near the edge is doing nothing wrong, and pulling it toward the
middle would be the Shield overriding a legal cruise.

```python
tgt = env.alt_min_m + (margin if state.up < env.alt_min_m else 0.0)
```

The margin is `min(1.0, band / 4)`. Recovery now completes in about twelve
seconds and the vehicle ends inside the band.

## What this says about the KPI set

The three KPIs this project had measured are all properties of **actions**:
did an illegal one fly, was the fail-safe correct, how often did the Shield
intervene. A Shield that repairs every action into another legal-but-useless one
scores perfectly on all three.

`mean time to safe` is the first that is a property of the **trajectory**, and
it found a real defect within an hour of existing. The related open item — the
wedge, where a subject sitting exactly on the route leaves every legal action
unable to make progress — is the same shape and is now pinned as
`standoff-wedge` in `experiments/scenarios.yaml`.

Both are the gap between *"the action was repaired"* and *"the trajectory was
sensible"*, and the second is not something a filter can guarantee on its own.
