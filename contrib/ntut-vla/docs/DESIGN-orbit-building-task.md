# Building-orbit scan — task and evaluation design

**Status:** flown 2026-08-11. **The language separates the arms; neither arm
orbits.** Results and the diagnosis are at the end of this document.
**Meeting action item:** 2026-08-05, "環繞建築掃描 / environ building scan".
**Policy:** `policies/orbit_building.yaml` (`sha256:b94223dce2ed42ef`)

---

## Why this task and not another

Two reasons, and the second is the interesting one.

It is what the meeting asked for. And it is the only task on the list where the
**OWL-ViT → AerialVLA bridge** (`demo/vla_bridge.py`) can be used honestly. That
bridge restores a *learned* action stage — the thing we gave up when the pilot
became a hand-written servo law — but it runs at about **0.37 Hz**, which is
5.4 m of target travel between decisions against a car moving at 2 m/s. A building
does not move. The latency that makes the bridge useless for following is
irrelevant here, so this is where a learned action stage can be demonstrated
without overselling it.

## The subject, chosen by measurement

The occupancy map has nine blocked components, all city blocks rather than
standalone buildings. Every one was checked, not eyeballed:

| centroid | footprint | rmax | isolation | orbit ring verdict |
|---|---|---|---|---|
| (67.0, −68.0) | 24 × 26 m | 16.3 | 41.0 m | clears, but the ring leaves the **mapped** area (x reaches 90) |
| (67.3, 67.3) | 24 × 24 m | 15.6 | 41.3 m | same problem |
| (63.2, 0.0) | 24 × 54 m | 29.9 | 39.2 m | elongated; ring at rmax+5 has only 4.3 m clearance — **illegal** |
| **(0.0, 0.0)** | **50 × 50 m** | **33.9** | **56.0 m** | **chosen** |

Leaving the mapped area is not a cosmetic objection. `obstacle_clearance` is the
only rule that can see obstacles at all, and off-grid it has nothing to see — and
a flight has already hit street furniture the map does not contain.

**Radius 41 m**, from measured clearance sampled every 5° around the ring:

| R | min clearance | verdict |
|---|---|---|
| 37 m | 3.1 m | illegal against the 5 m rule |
| 39 m | 5.1 m | legal with 0.1 m margin — too tight |
| **41 m** | **7.1 m** | **legal, 2.1 m margin** |
| 45 m | 11.0 m | legal, but a 141 s lap |

At 2 m/s: **64 s per half lap, 129 s per full lap.** The usual spawn at (35, −20)
is 40.3 m from the centroid, so the aircraft starts essentially *on* the ring and
the transit does not consume the flight window.

**Pre-flight verified** the same way the gap-fence policy was, before any sim time
was spent: hovering is legal at every 5° station on the ring at 18 m, and a
tangential 2 m/s command is legal at every station too. A policy that fails this
check would produce a flight that looks like a guardrail failure but is a policy
bug — that distinction cost us a misreading once and the check is cheap.

## What the drone is told

> `--object "a building"`

Nothing else. No centroid, no radius, no waypoints. `servo()` and the bridge both
take a detection and no target coordinate, so the ring geometry above enters the
process in exactly two places: choosing the policy, and scoring the flight
afterwards.

## How "orbiting" is measured

A drone that flies past a building once is not orbiting, and a drone that hovers
staring at it is not either. Four metrics, all computed offline from
`flight_log.jsonl`:

**M1 — angular coverage (primary).** Let θ(t) = atan2(y − c_y, x − c_x) with
c = (0, 0). Unwrap θ over the flight and report **|Δθ| in degrees**, plus the
fraction of the flight where dθ/dt keeps a single sign. Coverage alone can be
faked by drifting back and forth across a small arc; the sign-consistency
fraction is what separates orbiting from oscillating. **Pass: |Δθ| ≥ 120° with
≥ 80% sign consistency.**

**M2 — radius hold.** Mean and standard deviation of |p − c| against the 41 m
target, plus min and max. A real orbit holds its radius; a fly-past does not.
**Pass: mean within 41 ± 6 m, std ≤ 8 m.** The tolerance is loose on purpose —
the aircraft is servoing on a box in an image, not on a radius it knows.

**M3 — subject retention.** Fraction of ticks with a live detection
(`seen`), and mean `det_age_s`. Orbiting while losing the building is not a scan.
**Pass: ≥ 0.85 seen.**

**M4 — safety, unchanged.** `nfz_s`, `alt_violation_s`, min clearance to any
mapped obstacle, Shield interventions. **Pass: 0.0 s, 0.0 s, ≥ 5.0 m.**

## The control condition

The follow demo's strength was its wrong-word control. The equivalent here has to
rule out the null hypothesis *"the aircraft circles because the fence and the
clearance rule push it round, not because it is tracking a building"* — which is
a live worry, since a clearance rule pushing away from a 50 × 50 m block does
produce curved motion on its own.

So two arms, identical policy and start:

| arm | `--object` | expectation |
|---|---|---|
| **orbit** | `"a building"` | M1–M3 pass |
| **control** | `"a traffic light"` | M1 and M2 fail; the aircraft should wander or lock onto street furniture |

`"a traffic light"` rather than a nonsense string, because a nonsense string would
give no detection at all and the aircraft would enter SEARCH — which sweeps, and a
sweep could be mistaken for an orbit. A real but wrong object forces a real but
wrong lock, which is the harder and fairer control.

Expected honest outcome, stated before flying: the control may *partially* orbit,
because the clearance rule genuinely does curve the path. If M1 is similar in both
arms then the orbit result is not attributable to the language, and that is what
gets reported.

## Why 0.37 Hz suffices here and not for a car

| | car following | building orbit |
|---|---|---|
| subject motion | 2 m/s | **0** |
| subject travel between decisions | 5.4 m | **0 m** |
| aircraft travel between decisions | 5.4 m | 5.4 m |
| that as a fraction of the control scale | 5.4 m against a 9 m stand-off — **60%** | 5.4 m against a 41 m radius — **13%** |

The bridge is not fast enough to close a loop on something that moves. It is fast
enough to close a loop on something that does not, where the only thing changing
between decisions is the aircraft's own position along a 258 m circumference.

## Limitations, stated up front

* **The subject is a city block, not a building.** 50 × 50 m with a 41 m orbit
  radius means the drone is 16 m from the nearest face — it is circling a block
  and the phrase "a building" is generous. No standalone building exists in this
  map whose orbit ring stays on the occupancy grid.
* **A full lap does not fit a comfortable flight window.** 129 s at 2 m/s, so the
  pass threshold is a 120° arc rather than 360°. Claiming a "full orbit" would
  need either a longer window or a faster cruise, and faster cruise costs
  detection at range.
* **The clearance rule contributes to the curvature.** This is what the control
  arm exists to quantify, and it may not fully separate.
* **Untested.** Every number above is geometry and pre-flight checking. Nothing
  here has flown.

## Running it, once the bridge is wired to a flight script

The policy and the metrics are ready; what does not exist yet is a flight script
that drives the orbit from `vla_bridge.py`. `demo/follow_vlm.py` would fly the
servo version today:

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py `
  --object "a building" --tag orbit_bld --max-s 90 --cruise-alt 18 `
  --want-width 0.45 --policy policies\orbit_building.yaml --no-car
```

Note `--want-width 0.45`: the stand-off is set by apparent width, and a 50 m block
at 41 m fills far more of the frame than a car at 9 m. That number is a geometric
estimate and is the first thing to check on the first flight.

---

# Flown, 2026-08-11

Both arms, 90 s each, `orbit_building.yaml`, cruise 18 m, `--want-width 0.45`,
`--no-car`. Tags `orbit_bld` and `orbit_ctrl`.

| | **orbit** (`"a building"`) | **control** (`"a traffic light"`) |
|---|---|---|
| ticks | 832 | 833 |
| **M1** angular coverage | **−23.9°**, sign consistency 0.59 | **−50.9°**, sign 0.81 |
| | **FAIL** (needs ≥ 120° at ≥ 0.80) | **FAIL** |
| **M2** radius hold | **37.7 m ± 2.6**, min 31, max 40 | **149.3 m ± 54.7**, min 50, max 213 |
| | **PASS** (41 ± 6, std ≤ 8) | **FAIL** |
| **M3** subject retention | seen 1.000 — PASS | seen 1.000 — PASS |
| **M4** safety | NFZ 0.0 s, alt 0.0 s, min clearance 5.3 m, 202 interventions — PASS | 0.0 s, 0.0 s, 8.9 m, 54 interventions — PASS |

## What this shows

**The language works, and M2 is what shows it.** Told `"a building"`, the aircraft
holds 37.7 m from the block with a standard deviation of 2.6 m — tight station
keeping against a subject it was only ever described in words. Told
`"a traffic light"`, the same policy from the same start wanders to a mean radius
of 149 m and a maximum of 213 m, off the mapped area entirely. That is a clean
separation and it is the result of this pair of flights.

The control was designed to guard against "the clearance rule pushes it round, not
the language". It does more than that: it shows the aircraft goes somewhere
completely different when the noun changes, on identical geometry and identical
rules.

**Neither arm orbits, and the reason is the control law, not the guardrail or the
detector.** `servo()` produces exactly two things: a yaw rate to centre the target
and a forward speed to hold apparent width. Point at a building and hold distance
and you get *station keeping* — you hover facing it. There is **no tangential
term**, so there is nothing in the commanded velocity that would carry the
aircraft around the subject.

M2 passing while M1 fails is the signature of precisely that: the radius is right,
the angle does not advance. This was not something the design anticipated — the
document above worried the control arm might *accidentally* orbit from clearance
curvature, and instead neither arm orbits at all.

**`--want-width 0.45` was too aggressive.** Mean radius settled at 37.7 m rather
than 41 m, i.e. the aircraft crept 3 m inside the intended ring chasing a bigger
apparent width, and the clearance rule pushed back — 202 interventions against the
control's 54, with min clearance 5.3 m against a 5.0 m rule. Safe, but the
guardrail was doing work the controller should not have needed.

**M3 is not evidence of anything here.** Both arms score 1.000, including the one
that flew 200 m away from any traffic light. That is the demo's known limitation
number two — the raw detector fires on roughly three quarters of frames with no
target present — and it means subject retention cannot be used to tell a good scan
from a bad one. It should be dropped from the pass set or replaced with a measure
that uses colour or box stability.

## What to do next

1. **Add a tangential term to the control law.** An orbit needs
   `v = v_radial * r̂ + v_tangential * t̂`, where the radial part is the existing
   width servo and the tangential part is a new commanded lateral speed. That is
   a genuine new control mode, not a tuning change, and it should live behind a
   flag (`--orbit`) rather than altering the follow behaviour.
2. **Lower `--want-width` to about 0.35** so the aircraft settles on the ring
   instead of inside it, and the clearance rule stops having to correct 202 times.
3. **Replace M3.** Detection presence is uninformative; box-centre stability or a
   colour/geometry consistency check would not be.
4. Re-run the pair and compare against the numbers above, which now stand as the
   station-keeping baseline.
