"""Every known person, for a distance_envelope over `people` (WP1-07).

WHY THIS EXISTS

The grant's distance_envelope is a "min distance to typed object class
(people, buildings, roads)" (policy-dsl.md, the constraint taxonomy). Until
2026-11-18 the people class was enforced as a stand-off from ONE position:
the tracked subject (Shield.set_subject), and only while perception called it
a person. A person who was not the subject - one of the 40 pedestrians
walking CityLife beside a car follow - was outside the rule, and a car follow
under a people envelope never bound it at all (models.DistanceEnvelope, "THE
RECORDED LIMIT").

Since 2026-11-18 the people class binds every person the Shield is told
about, through the same clearance runtime buildings and roads use, over a
field this module builds:

    PeopleField    the signed distance from a point to the nearest person's
                   reach disc (below); what Shield._distance_at reads for a
                   people rule, exactly where it reads the obstacle or road
                   map for the other two classes;
    PeopleMemory   the rail side: per-frame person fixes associated into
                   tracks, a track unseen for more than PEOPLE_STALE_S
                   dropped (the staleness rule);
    Person         one person as the Shield takes it: a position and the
                   age of the fix it comes from.

WHERE THE PEOPLE COME FROM (each rail names its source; the Shield refuses a
people envelope without one, as it refuses a buildings envelope without an
obstacle map - never inert)

    the PAS / CityLife perception rail   every box the detector scores as a
        person in each frame (follow_vlm.Grounder's people phrase, asked in
        the subject's own forward pass), ground-projected with the geometry
        the follow places its subject with (perception_node.subject_estimate:
        the box column's bearing, the capture's depth inside the box, else
        the box width at a person's width, from the capture's pose);
    the city sim rails (follow_vlm, vla_fly)   the simulator's pedestrian
        list: the level's own crowd polled by tag (level_actors.LevelActors,
        --level-peds) or the client's spawned figures (pedestrians.py);
    the SITL rails and the headless harness   the declared subject only, when
        it is a person (there are no pedestrians in those worlds) - recorded
        as that, not as "every person".

The tracked subject is always one of the people when it is classified as a
person (models.PEOPLE_CLASSES): the Shield adds it to the field itself, so a
rail that knows only its subject still binds the rule to it.

THE NUMBERS (documented, and each a module constant a caller may override)

    PEOPLE_STALE_S = 3.0 s   a track unseen for longer is dropped. The PAS
        detector publishes about 3 frames a second (the 287 ms in-flight
        inference, follow_vlm.Grounder) through a 90 deg field of view, so a
        person stays in memory for about nine detector frames after the last
        one that saw it: long enough to bridge a missed frame or a short
        occlusion, short enough that a phantom does not hold the aircraft off
        a street for long.
    PERSON_SPEED_MPS = 1.5 m/s   how fast a person may have walked since the
        fix: a person seen `age` seconds ago is a disc of radius
        PERSON_SPEED_MPS * age about the fix (its reach), so the rule holds
        min_distance_m from everywhere the person could be by now, not from
        where it was (1.4 m/s is the walking speed pedestrians.py animates).
    PEOPLE_ASSOC_M = 2.0 m   a fix within this distance (plus the reach of the
        track's age) of a track is the same person (greedy nearest-first,
        one fix per track per frame); farther, a new track.

What the field does NOT model, stated: a person's motion during the
clearance forecast itself (the forecast reads the field as it stands this
tick, as it reads a building); people the source does not report (outside
the camera's view and older than PEOPLE_STALE_S; below the detector's
threshold; not in the simulator's list); and height - the distance is
horizontal, as for buildings and roads, so an aircraft directly above a
person at any altitude is at distance 0."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

PEOPLE_STALE_S = 3.0
PERSON_SPEED_MPS = 1.5
PEOPLE_ASSOC_M = 2.0
# The rails hand the Shield only the people whose reach disc comes within
# a cull range of the aircraft (`within`): the cull keeps the per-tick cost
# from growing with a crowd spread over the whole 2 km level (the Shield's
# distance query is a loop over the people it holds). The range is sized
# per policy and forecast (`cull_range_m`): the envelope's distance plus the
# farthest the forecast can carry the aircraft, plus a margin, and never
# less than this floor. The grant's 5 s lookahead at the 5 m/s cap of the
# CityLife follow policies reaches 25 m, so every shipped people policy
# (4 m) culls at the floor.
PEOPLE_RANGE_M = 100.0
# What cull_range_m keeps beyond the envelope distance plus the forecast's
# reach: the clearance repair's gradient probe (PEOPLE_FIELD_RES_M / 2) and
# one control tick of the aircraft's own motion are well inside it.
PEOPLE_RANGE_MARGIN_M = 10.0
# The step the clearance repairs probe a field's gradient with is half its
# cell size (Shield._away_dir); the people field has no cells, and 0.5 m
# gives a 0.25 m probe, well inside a disc of the smallest sensible envelope.
PEOPLE_FIELD_RES_M = 0.5


@dataclass(frozen=True)
class Person:
    """One person as the Shield takes it (Shield.set_people): where the fix
    put it (x North, y East, the Action4D frame) and how old that fix is."""
    x: float
    y: float
    age_s: float = 0.0

    def reach_m(self, speed_mps: float = PERSON_SPEED_MPS) -> float:
        """How far the person may have walked since the fix."""
        return speed_mps * max(0.0, self.age_s)


class PeopleField:
    """Signed metres from a point to the nearest person's reach disc:
    min over people of (|p - fix| - reach), negative inside a disc; inf with
    nobody. 1-Lipschitz, like the map fields (Shield._field_lipschitz)."""
    __slots__ = ("pts",)

    def __init__(self, pts: Iterable[tuple[float, float, float]] = ()):
        self.pts = tuple((float(x), float(y), float(r)) for x, y, r in pts)

    def __len__(self) -> int:
        return len(self.pts)

    def distance(self, x: float, y: float) -> float:
        if not self.pts:
            return math.inf
        return min(math.hypot(x - px, y - py) - r for px, py, r in self.pts)


def as_person(p) -> Person:
    """A Person from a Person, an (x, y) or an (x, y, age_s)."""
    if isinstance(p, Person):
        return p
    if len(p) == 2:
        return Person(float(p[0]), float(p[1]))
    return Person(float(p[0]), float(p[1]), float(p[2]))


def is_finite_person(p: Person) -> bool:
    return math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.age_s)


class PeopleMemory:
    """Per-frame person fixes into tracks, with the staleness rule.

    observe(t_seen, points, key): one frame's fixes, all taken at `t_seen`
    (the frame's capture time, on the clock `current` is asked on). A frame
    already observed (the same `key` as the last call: a detector frame read
    again on the next control tick) is ignored. Each fix goes to the nearest
    unmatched track within PEOPLE_ASSOC_M plus that track's reach since its
    last fix, nearest pairs first; the rest start tracks. A non-finite fix is
    dropped and counted.

    current(t_now): the tracks seen within `stale_s` of `t_now`, as Persons
    whose age is t_now - their last fix; older tracks are dropped (counted).
    """

    def __init__(self, stale_s: float = PEOPLE_STALE_S, assoc_m: float = PEOPLE_ASSOC_M,
                 speed_mps: float = PERSON_SPEED_MPS):
        for name, v in (("stale_s", stale_s), ("assoc_m", assoc_m), ("speed_mps", speed_mps)):
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError(f"PeopleMemory {name} must be a positive number, got {v!r}")
        self.stale_s = float(stale_s)
        self.assoc_m = float(assoc_m)
        self.speed_mps = float(speed_mps)
        self._tracks: list[list[float]] = []      # [x, y, t_seen]
        self._last_key = None
        self.n_frames = 0
        self.n_fixes = 0
        self.n_nonfinite = 0
        self.n_tracks = 0
        self.n_dropped_stale = 0
        self.max_live = 0

    def observe(self, t_seen: float, points, key=None) -> int:
        """One frame's fixes; returns how many started a new track."""
        if key is not None and key == self._last_key:
            return 0
        self._last_key = key
        t_seen = float(t_seen)
        if not math.isfinite(t_seen):
            raise ValueError(f"observe: t_seen must be finite, got {t_seen!r}")
        fixes = []
        for p in points:
            x, y = float(p[0]), float(p[1])
            if math.isfinite(x) and math.isfinite(y):
                fixes.append((x, y))
            else:
                self.n_nonfinite += 1
        self.n_frames += 1
        self.n_fixes += len(fixes)
        pairs = []
        for i, (x, y) in enumerate(fixes):
            for j, tr in enumerate(self._tracks):
                gate = self.assoc_m + self.speed_mps * max(0.0, t_seen - tr[2])
                d = math.hypot(x - tr[0], y - tr[1])
                if d <= gate:
                    pairs.append((d, i, j))
        pairs.sort()
        used_i, used_j = set(), set()
        for _, i, j in pairs:
            if i in used_i or j in used_j:
                continue
            used_i.add(i)
            used_j.add(j)
            tr = self._tracks[j]
            if t_seen >= tr[2]:
                tr[0], tr[1], tr[2] = fixes[i][0], fixes[i][1], t_seen
        new = [(x, y) for i, (x, y) in enumerate(fixes) if i not in used_i]
        for x, y in new:
            self._tracks.append([x, y, t_seen])
        self.n_tracks += len(new)
        self.max_live = max(self.max_live, len(self._tracks))
        return len(new)

    def current(self, t_now: float) -> list[Person]:
        """The live tracks at `t_now` (the staleness rule applied first)."""
        t_now = float(t_now)
        keep = [tr for tr in self._tracks if t_now - tr[2] <= self.stale_s]
        self.n_dropped_stale += len(self._tracks) - len(keep)
        self._tracks = keep
        return [Person(tr[0], tr[1], max(0.0, t_now - tr[2])) for tr in keep]

    def record(self) -> dict:
        """What a flight's record keeps of the memory."""
        return {"stale_s": self.stale_s, "assoc_m": self.assoc_m,
                "speed_mps": self.speed_mps, "frames": self.n_frames,
                "fixes": self.n_fixes, "nonfinite": self.n_nonfinite,
                "tracks": self.n_tracks, "dropped_stale": self.n_dropped_stale,
                "max_live": self.max_live}


def within(people, x: float, y: float, range_m: float = PEOPLE_RANGE_M) -> list[Person]:
    """The people whose reach disc comes within `range_m` of (x, y): what a
    rail hands Shield.set_people for an aircraft at (x, y). The rails pass
    the range cull_range_m sizes for their policy and forecast; the default
    is its floor, PEOPLE_RANGE_M."""
    out = []
    for p in people:
        q = as_person(p)
        if math.hypot(q.x - x, q.y - y) - q.reach_m() <= range_m:
            out.append(q)
    return out


def needs_people(policy) -> list[str]:
    """The ids of `policy`'s distance_envelope rules over people."""
    from .models import DistanceEnvelope
    return [c.id for c in policy.by_type(DistanceEnvelope) if c.object_class == "people"]


def cull_range_m(policy, lookahead_s: float, speed_mps: float | None = None) -> float:
    """The range a rail hands people within (`within`) for `policy` judged
    over a `lookahead_s` forecast (review 2026-11-18: a fixed 100 m cull
    was checked against nothing).

    A person matters to a tick when its reach disc comes within the
    envelope's min_distance_m of a point the Shield judges: the aircraft and
    its forecast. The forecast flies at most the policy's horizontal speed
    cap (the kinematic_envelope's tightest speed_max_mps: the emitted
    action is clamped to it) or the rail's own commanded maximum
    (`speed_mps`, for the raw action's forecast), whichever is larger, for
    lookahead_s (an upper bound: a clearance rule's forecast is the
    stopping horizon, never longer than the lookahead). So the range is

        max(min_distance_m) + lookahead_s * speed + PEOPLE_RANGE_MARGIN_M,

    and never less than PEOPLE_RANGE_M (the cull every rail flew with:
    4 + 5 * 5 + 10 = 39 m for the shipped CityLife people policy). With no
    speed bound at all - no cap in the policy and none given - nothing
    bounds the forecast, and the answer is inf: no cull, every person the
    source reports is handed over. A policy without a people rule: the
    floor (nobody is handed over anyway)."""
    from .models import DistanceEnvelope, KinematicEnvelope
    dists = [c.min_distance_m for c in policy.by_type(DistanceEnvelope)
             if c.object_class == "people"]
    if not dists:
        return PEOPLE_RANGE_M
    if not (isinstance(lookahead_s, (int, float)) and math.isfinite(lookahead_s)
            and lookahead_s >= 0):
        raise ValueError(f"cull_range_m: lookahead_s must be a finite number >= 0, "
                         f"got {lookahead_s!r}")
    speeds = [c.speed_max_mps for c in policy.by_type(KinematicEnvelope)]
    cap = min(speeds) if speeds else None
    if speed_mps is not None and not (math.isfinite(speed_mps) and speed_mps >= 0):
        raise ValueError(f"cull_range_m: speed_mps must be a finite number >= 0, "
                         f"got {speed_mps!r}")
    bound = [v for v in (cap, speed_mps) if v is not None]
    if not bound:
        return math.inf
    need = max(dists) + float(lookahead_s) * max(bound) + PEOPLE_RANGE_MARGIN_M
    return max(PEOPLE_RANGE_M, need)


def range_record(range_m: float):
    """cull_range_m's answer as a flight record writes it: the metres, or
    the words for no cull (an inf is not valid JSON)."""
    return (round(float(range_m), 2) if math.isfinite(range_m)
            else "no cull: neither the policy nor the rail bounds the speed")


def row_people(row: dict) -> list[Person] | None:
    """The people a flight log row says its Shield was given (`people`:
    [[x, y, age_s], ...], rows_people), as Persons; None when the row has no
    `people` field (a rail that fed none, or a row before 2026-11-18)."""
    ppl = row.get("people")
    if ppl is None:
        return None
    return [as_person(p) for p in ppl]


def logged_people_source(policy, rows, metrics: dict | None) -> tuple[dict, list[str]]:
    """For an OFFLINE reader that rebuilds a flight's Shield from its log
    (tools/rescore_kpis.py, tools/rerender_policy_hud.py):
    `({"people_source": ...} or {}, problems)`.

    No distance_envelope over people: ({}, []). One, and every row with a
    pose carries `people` (the rails that feed people log what each tick's
    Shield was given, rows_people): the source is that log, replayed with
    `row_people` on each tick, named with the flight's own source
    (metrics.people_feed.source). One on a rail whose only person was the
    declared subject (the SITL rails, the headless harness: metrics
    `people_source`, or a declared subject classified as a person): that
    subject, which the Shield adds itself. Otherwise no source and the
    reason - the reader refuses with it rather than judge a people rule over
    nobody (a rule that never ran reads as a clean flight)."""
    from .models import PEOPLE_CLASSES
    ids = needs_people(policy)
    if not ids:
        return {}, []
    metrics = metrics or {}
    posed = [r for r in rows if r.get("x") is not None and r.get("y") is not None]
    logged = [r for r in posed if r.get("people") is not None]
    flown = (metrics.get("people_feed") or {}).get("source") \
        if isinstance(metrics.get("people_feed"), dict) else None
    if logged:
        if len(logged) < len(posed):
            return {}, [f"{', '.join(ids)}: {len(posed) - len(logged)} of {len(posed)} "
                        f"ticks with a pose have no `people` field, so the people "
                        f"their Shield was given are not in the log"]
        return ({"people_source": "replayed from the flight log's per-row `people`"
                                  + (f" (as flown: {flown})" if flown else "")}, [])
    declared = metrics.get("people_source")
    subj = metrics.get("subject") if isinstance(metrics.get("subject"), dict) else None
    if isinstance(declared, str) and declared.startswith("the declared subject only"):
        return {"people_source": f"replayed: {declared}"}, []
    if subj is not None and str(subj.get("class") or "").lower() in PEOPLE_CLASSES \
            and subj.get("x") is not None and subj.get("y") is not None:
        return ({"people_source": f"replayed: the declared subject only "
                                  f"({subj.get('class')} at ({float(subj['x']):.1f}, "
                                  f"{float(subj['y']):.1f}))"}, [])
    return {}, [f"{', '.join(ids)}: a distance_envelope over people, and the log "
                f"records no people (no per-row `people`, no declared person "
                f"subject){'' if not flown else f'; the flight named {flown!r}'}"]


def declared_subject_source(policy, subject, subject_class: str | None,
                            rail: str) -> tuple[dict, list[str]]:
    """For a rail whose world has no pedestrians (the ArduPilot SITL rails,
    the headless harness): `({"people_source": ...} or {}, problems)`.

    No distance_envelope over people: ({}, []) - the Shield is built as
    before. One, with a declared subject classified as a person: the source
    is that subject alone (the Shield adds it to the people field itself),
    said so in the words recorded. One with no such subject: no source and
    the reason, which the rail refuses with (a people rule judged over
    nobody would be a rule that never ran)."""
    from .models import PEOPLE_CLASSES
    ids = needs_people(policy)
    if not ids:
        return {}, []
    if subject is not None and subject_class is not None \
            and subject_class.lower() in PEOPLE_CLASSES:
        try:
            at = f" at ({float(subject[0]):.1f}, {float(subject[1]):.1f})"
        except (TypeError, IndexError, ValueError):
            at = ""          # a subject whose position is resolved later
        return ({"people_source": f"the declared subject only ({subject_class}{at}; "
                                  f"{rail} has no pedestrians)"}, [])
    return {}, [f"{i}: a distance_envelope over people, and {rail} has no person to "
                f"judge it against (no pedestrians; the declared subject is "
                f"{'absent' if subject is None else repr(subject_class)})" for i in ids]


def figure_points(figures) -> list[tuple[float, float]]:
    """(x, y) of each simulator figure that is a usable fix this tick: a
    level actor that has resolved and whose last poll was finite
    (level_actors.LevelFigure: seen, stale == 0), or a client figure whose
    spawn did not fail (pedestrians.Figure: _failed)."""
    out = []
    for f in figures:
        if not getattr(f, "seen", True) or getattr(f, "stale", 0) or getattr(f, "_failed", False):
            continue
        x, y = float(f.x), float(f.y)
        if math.isfinite(x) and math.isfinite(y):
            out.append((x, y))
    return out


def rows_people(people, ndigits: int = 2) -> list[list[float]]:
    """The people a tick's Shield was given, for the flight log: [x, y, age_s]."""
    return [[round(p.x, ndigits), round(p.y, ndigits), round(p.age_s, 3)]
            for p in (as_person(q) for q in people)]
