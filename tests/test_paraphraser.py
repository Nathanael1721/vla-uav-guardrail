"""guardrail/paraphraser.py - the free-form wording service (WP2-13, WP4-10, ARCH-23).

Run either way:
    pytest tests/test_paraphraser.py -v
    python tests/test_paraphraser.py

WHY THIS FILE EXISTS

The grant names a Paraphraser as a Q3 deliverable, separate from the Prefix
Compiler ("Free-form paraphrasing is the paraphraser's job (Q3 deliverable,
separate service)", Prefix Compiler PDF p5), and WP4 owns a per-paraphrase
robustness KPI (Grant overview PDF p2, WP table). A robustness number is only as
good as the paraphrases behind it: a "paraphrase" that quietly turned 4 m/s
into 5 m/s, or "at least 10 m" into "at most 10 m", would measure a different
mission and call the difference fragility.

So most of these tests are about REFUSAL, the same way tests/test_bundle.py is:

  * the validator must say no to every way of changing a mission slot it reads -
    a number, a unit, a bound, a zone, the polarity of a zone, a colour and the
    object it sits on, an object, a direction, a place and its role, a
    coordinate, the action, a negation added or dropped, a condition or an
    exception - and to the null, the identity "paraphrase", which changes
    nothing and so tests nothing;
  * the store must say no to a file whose text, id, source, order or uniqueness
    was tampered with;
  * the stored backend must say no to a source it has no set for, instead of
    quietly falling back to something else.

Then the positive half: same seed -> same paraphrases, ids that do not move,
every stored paraphrase passing the validator, and every rule text a pilot can
be shown (compile_csp's rendering as well as build_prompt's) served.

What these tests do NOT show: that the validator catches meaning changes of a
kind nobody wrote down. The held-out probe rates (test_probe_rates_*) are the
honest measure of that, and they are not zero.
"""
import json
import logging
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import paraphraser as P                              # noqa: E402

STORE = ROOT / "experiments" / "paraphrases"

CSP_DEMO = ("Never enter zone 'nfz-square'. Stay between 10 m and 20 m altitude. "
            "Keep speed at or below 4 m/s, climb rate below 2 m/s and turn rate "
            "below 45 deg/s.")
CSP_PED = ("Stay between 10 m and 20 m altitude. Keep speed at or below 4 m/s, "
           "climb rate below 2 m/s and turn rate below 45 deg/s. Keep at least "
           "10 m away from any pedestrian. Keep at least 5 m away from anything "
           "you are following.")
CSP_CORRIDOR = ("Never enter zone 'nfz-school' (in force Mon-Fri 07:30-17:30). Stay inside "
                "corridor 'corridor-survey-route', within 20 m of its centerline and between "
                "10 m and 20 m. Keep speed at or below 4 m/s, climb rate below 2 m/s "
                "and turn rate below 45 deg/s.")
TASK_PAD = "fly to the northeast pad at 6 m/s"
TASK_CAR = "follow a red car"
TASK_FWD = "fly forward and avoid restricted areas"
TASK_XY = "fly to (40, 40) at 6 m/s altitude 20"
TASK_WP = "Fly to the waypoint 120 m ahead at cruise altitude."
REST = (" Hold altitude between 10 m and 20 m. Max speed 4 m/s; climb under 2 m/s; "
        "turn under 45 deg/s.")


def _refused(src, cand, code=None):
    v = P.validate(src, cand)
    assert not v.ok, f"validator ACCEPTED a slot change:\n  src : {src}\n  cand: {cand}"
    if code is not None:
        assert any(r.startswith(code) for r in v.reasons), \
            f"refused, but not for {code!r}: {v.reasons}"
    return v


def _accepted(src, cand):
    v = P.validate(src, cand)
    assert v.ok, f"validator REFUSED a faithful paraphrase:\n  src : {src}\n  " \
                 f"cand: {cand}\n  why : {v.reasons}"
    return v


_LIVE = {}


def _live_report():
    """validate_store() recomputed now (12 s), once per run: the mutation test
    and the report-consistency test read the same numbers."""
    if "rep" not in _LIVE:
        _LIVE["rep"] = P.validate_store(STORE)
    return _LIVE["rep"]


# --------------------------------------------------------------------------- #
# The null: the identity "paraphrase" is not a paraphrase
# --------------------------------------------------------------------------- #

def test_identity_is_flagged_as_not_a_paraphrase():
    """The zero-skill answer. A paraphraser that returned its input would pass
    every slot check there is - nothing was dropped, nothing was changed - and a
    robustness KPI computed over it would report perfect robustness having
    tested nothing at all."""
    for src in (CSP_DEMO, CSP_PED, TASK_PAD, TASK_CAR, TASK_FWD, TASK_XY):
        _refused(src, src, "identity")


def test_case_and_punctuation_changes_are_still_identity():
    _refused(CSP_DEMO, CSP_DEMO.upper(), "identity")
    _refused(TASK_CAR, "Follow a red car.", "identity")
    _refused(TASK_PAD, "  fly to the northeast pad,  at 6 m/s!  ", "identity")


def test_a_one_word_change_in_a_long_rule_text_is_near_identity():
    """Below the novelty floor: one word in thirty is not a different wording."""
    cand = CSP_DEMO.replace("Never enter", "Do not enter")
    _refused(CSP_DEMO, cand, "identity")


def test_every_stored_source_refuses_its_own_identity():
    store = P.load_store(STORE)
    assert store, "no stored paraphrase sets found"
    for entry in store.values():
        v = P.validate(entry.source_text, entry.source_text)
        assert not v.ok and any(r.startswith("identity") for r in v.reasons), \
            entry.source_key


def test_canonical_arm_is_labelled_identity_not_a_paraphrase():
    """The per-paraphrase KPI needs the canonical wording as its baseline arm.
    It gets an id so it can be logged, but its backend says what it is."""
    c = P.canonical(TASK_CAR)
    assert c.backend == "identity" and c.text == TASK_CAR
    assert c.paraphrase_id == P.make_paraphrase_id(TASK_CAR, "identity", 0, TASK_CAR)
    assert not P.validate(TASK_CAR, c.text).ok


# --------------------------------------------------------------------------- #
# Refusals: every mission slot, mutated one at a time
# --------------------------------------------------------------------------- #

def test_refuses_a_changed_number():
    _refused(CSP_DEMO, "Stay out of zone 'nfz-square'. Hold altitude between 10 m "
             "and 20 m. Max speed 5 m/s; climb under 2 m/s; turn under 45 deg/s.",
             "quantity")
    _refused(TASK_PAD, "head over to the northeast pad, cruising at 7 m/s", "quantity")


def test_refuses_a_number_written_as_a_different_word():
    _refused(TASK_PAD, "head over to the northeast pad, cruising at seven metres "
             "per second", "quantity")


def test_refuses_a_changed_unit():
    _refused(TASK_PAD, "head over to the northeast pad, cruising at 6 km/h", "quantity")
    _refused(CSP_PED, CSP_PED.replace("10 m away", "10 ft away")
             .replace("Stay between", "Fly between"), "quantity")


def test_refuses_a_flipped_bound():
    """Same number, same unit, opposite meaning: the case a digits-only check
    cannot see."""
    good = ("Keep at least 10 m from pedestrians and at least 5 m from your target. "
            "Fly between 10 m and 20 m up. Max speed 4 m/s, climb under 2 m/s, turn "
            "under 45 deg/s.")
    _accepted(CSP_PED, good)
    _refused(CSP_PED, good.replace("at least 10 m", "at most 10 m"), "quantity")
    _refused(CSP_PED, good.replace("climb under 2", "climb over 2"), "quantity")
    _refused(CSP_DEMO, "Never fly into zone 'nfz-square'. Hold your altitude between "
             "10 m and 20 m. Never go slower than 4 m/s, climb under 2 m/s and turn "
             "under 45 deg/s.", "quantity")


def test_refuses_a_dropped_negation():
    good = ("Please do not fly into the zone called 'nfz-square'. Keep your altitude "
            "between 10 m and 20 m, and keep your speed to no more than 4 m/s, your "
            "climb rate under 2 m/s and your turn rate under 45 deg/s.")
    _accepted(CSP_DEMO, good)
    _refused(CSP_DEMO, good.replace("do not fly", "fly"), "polarity")
    _refused(CSP_DEMO, good.replace("no more than", "more than"), "quantity")


def test_refuses_quantities_swapped_between_rules():
    """Numbers, units and bounds all survive; only what they limit moved."""
    _refused(CSP_DEMO, "Stay out of zone 'nfz-square'. Hold altitude between 10 m and "
             "20 m. Keep your climb rate at or below 4 m/s and your speed below "
             "2 m/s, and turn below 45 deg/s.", "quantity")
    _refused(CSP_PED, "Fly between 10 m and 20 m up. Max speed 4 m/s, climb under "
             "2 m/s, turn under 45 deg/s. Keep at least 5 m from pedestrians and at "
             "least 10 m from your target.", "quantity")


def test_refuses_a_dropped_or_renamed_zone():
    good = "Stay out of zone 'nfz-square'." + REST
    _accepted(CSP_DEMO, good)
    _refused(CSP_DEMO, good.replace("zone 'nfz-square'", "the square"), "zone")
    _refused(CSP_DEMO, good.replace("nfz-square", "nfz-squares"), "zone")


def test_a_known_zone_id_counts_without_its_quotes():
    """The source's ids are matched however a paraphrase writes them; a
    near-miss spelling is a different zone."""
    _accepted(CSP_DEMO, "Keep out of the nfz-square zone." + REST)
    _refused(CSP_DEMO, "Keep out of the nfz-squares zone." + REST, "zone")


def test_refuses_a_zone_whose_polarity_flipped():
    good = "Stay out of zone 'nfz-square'." + REST
    _refused(CSP_DEMO, good.replace("Stay out of", "Stay inside"), "polarity")
    corr = ("Keep out of zone 'nfz-school' on weekdays from 07:30 to 17:30. Fly only "
            "inside corridor 'corridor-survey-route', no more than 20 m off its "
            "centerline and between 10 m and 20 m up. Max speed 4 m/s, climb under "
            "2 m/s, turn under 45 deg/s.")
    _accepted(CSP_CORRIDOR, corr)
    _refused(CSP_CORRIDOR, corr.replace("Fly only inside", "Fly only outside"),
             "polarity")
    _refused(CSP_CORRIDOR, corr.replace("Fly only inside corridor",
                                        "Never enter corridor"), "polarity")
    # the label word is not the slot: same id, same stay-inside polarity
    _accepted(CSP_CORRIDOR, corr.replace("Fly only inside corridor",
                                         "Never leave zone"))
    # "inside or near" no longer confines
    _refused(CSP_CORRIDOR, corr.replace("Fly only inside corridor",
                                        "Fly inside or near corridor"), "polarity")


def test_refuses_a_restricted_area_rule_that_no_longer_prohibits():
    _accepted(TASK_FWD, "fly straight ahead and keep out of restricted areas")
    _refused(TASK_FWD, "fly straight ahead and head into restricted areas", "polarity")
    _refused(TASK_FWD, "fly straight ahead and keep out of open areas", "object")


def test_refuses_permission_where_there_was_a_prohibition():
    """Independent review, 2026-10-06: every one of these passed as a keep-out
    rule, because any negator or prohibition word in the clause made it one."""
    _refused(CSP_DEMO, "It is not forbidden to enter zone 'nfz-square'." + REST, "polarity")
    _refused(CSP_DEMO, "You are allowed to enter zone 'nfz-square'." + REST, "polarity")
    _refused(CSP_DEMO, "You can fly into zone 'nfz-square' briefly." + REST, "polarity")
    _refused(TASK_FWD, "fly forward and do not avoid restricted areas", "polarity")
    _refused(TASK_FWD, "fly ahead, and never steer clear of restricted areas", "polarity")
    # ... while the faithful forms of the same words still pass
    _accepted(CSP_DEMO, "You may not enter zone 'nfz-square'." + REST)
    _accepted(CSP_DEMO, "Entering zone 'nfz-square' is not allowed." + REST)
    _accepted(CSP_DEMO, "Entry into zone 'nfz-square' is prohibited." + REST)
    _accepted(CSP_DEMO, "Zone 'nfz-square' is a no-go area." + REST)
    _accepted(CSP_DEMO, "'nfz-square' is a no-fly zone, so never enter it." + REST)


def test_a_second_clause_about_the_zone_counts():
    """'zone X is closed ..., so never go in there' rules twice on one zone; a
    second clause that permits entry ('so go in there') vetoes the first. Found
    by the mutation test on 2026-10-06 (a dropped 'never' passed)."""
    src = ("Stay between 5 m and 80 m altitude. Never enter zone 'nfz-school-yard' (in "
           "force Mon-Fri 07:30-17:30). Keep speed at or below 12 m/s, climb rate below "
           "4 m/s and turn rate below 60 deg/s.")
    good = ("The band you have to fly in runs from 5 m to 80 m of altitude. Zone "
            "'nfz-school-yard' is closed to you on weekdays from 07:30 to 17:30, so never "
            "go in there during those hours. Speed must not go above 12 m/s, climb rate "
            "must stay under 4 m/s and turn rate under 60 deg/s.")
    _accepted(src, good)
    _refused(src, good.replace("so never go in there", "so go in there"), "polarity")


def test_a_zone_rule_the_validator_cannot_read_fails_closed():
    """Found 2026-10-06: a too-broad entry word made the compiler's '(in force
    ...)' read as permission, the SOURCE zone lost its polarity, and with it
    every polarity check - two flipped mutants passed. An unreadable source
    ruling is now a refusal, not a pass."""
    src = "Zone 'nfz-x' is on the map. Keep speed at or below 4 m/s."
    _refused(src, "There is a zone called 'nfz-x' on the map. Max speed 4 m/s.", "polarity")


def test_refuses_a_changed_colour_or_object():
    _accepted(TASK_CAR, "track a red car")
    _refused(TASK_CAR, "track a blue car", "colour")
    _refused(TASK_CAR, "track a red truck", "object")
    _refused(TASK_CAR, "track a red vehicle", "object")
    _refused(TASK_CAR, "track a car", "colour")


def test_refuses_an_added_object():
    """No test pinned this until the 2026-10-06 review deleted the check in a
    mirror and all 53 tests still passed."""
    _refused(TASK_CAR, "follow a red car and a person", "object")


def test_the_colour_must_stay_on_its_object():
    _accepted(TASK_CAR, "keep the red car in sight and follow it")
    _accepted(TASK_CAR, "stay behind a red car and follow it")
    _refused(TASK_CAR, "follow the car that is parked next to the red one", "colour")
    _refused(TASK_CAR, "follow a red car, or any car", "colour")


def test_refuses_a_changed_direction():
    _refused(TASK_FWD, "fly backwards and keep out of restricted areas", "direction")
    _refused(TASK_FWD, "fly left and keep out of restricted areas", "direction")


def test_refuses_a_negated_action_or_direction():
    """Review, 2026-10-06: 'do not follow a red car' passed - the action word
    was there, so the action slot matched."""
    for cand in ("do not follow a red car", "stop following a red car",
                 "never track a red car"):
        _refused(TASK_CAR, cand, "action")
    _refused(TASK_FWD, "do not fly forward, and avoid restricted areas", "direction")
    _refused("stay with a person", "stay alongside a person but do not keep them in view",
             "action")
    _refused(TASK_WP, "fly to the waypoint lying 120 m ahead of you and do not hold "
             "cruise altitude on the way", "reference")


def test_refuses_a_changed_place_coordinate_or_place_role():
    _refused(TASK_PAD, "head over to the north pad, cruising at 6 m/s", "place")
    _refused(TASK_PAD, "head over to the north-east pad, cruising at 6 m/s", "place")
    _refused(TASK_XY, "head to (40, 30) at 6 m/s, holding altitude 20", "coordinate")
    _refused(TASK_XY, "head to (40, 40) at 6 m/s, holding altitude 25", "quantity")
    _refused(TASK_PAD, "fly to a spot far away from the northeast pad at 6 m/s", "place")
    _refused(TASK_PAD, "fly past the northeast pad at 6 m/s", "place")
    # a quantity between the verb and "to" is still a goto
    _accepted(TASK_PAD, "fly at 6 m/s to the northeast pad")


def test_refuses_a_changed_action():
    _refused(TASK_CAR, "find a red car", "action")
    _refused(TASK_PAD, "circle the northeast pad at 6 m/s", "action")
    _refused(TASK_PAD, "fly toward the northeast pad at 6 m/s but stop halfway", "action")


def test_refuses_an_added_rule():
    """A paraphrase may not invent a constraint the policy does not contain."""
    _refused(TASK_CAR, "track a red car and stay below 20 m", "quantity")
    _refused(TASK_FWD, "fly straight ahead and keep out of restricted areas and "
             "zone 'nfz-park'", "zone")


def test_refuses_a_narrowed_follow_subject():
    """Found on review on 2026-10-06, after the store had passed 96/96: seven
    stored paraphrases rendered the stand-off wildcard "anything you are
    following" as "the person you are following", "the car you are following",
    "whoever you are following". Right on the mission they were written for,
    wrong the day the same policy is flown behind something else - and the
    validator, which read every followed-subject phrase as one slot, accepted
    all seven."""
    base = ("Fly between 10 m and 20 m up. Max speed 4 m/s, climb under 2 m/s, turn "
            "under 45 deg/s. Keep at least 10 m from pedestrians and at least 5 m "
            "from {who}.")
    for who in ("whatever you are following", "your target", "the subject you follow",
                "the thing it is tracking", "the one you are following"):
        _accepted(CSP_PED, base.format(who=who))
    for who in ("the person you are following", "whoever you are following",
                "the car you are following", "the vehicle it is following",
                "the tracked car", "anyone you're tailing"):
        _refused(CSP_PED, base.format(who=who), "object")


def test_refuses_a_dropped_sentence():
    full = ("Fly between 10 m and 20 m up. Max speed 4 m/s, climb under 2 m/s, turn "
            "under 45 deg/s. Keep at least 10 m from pedestrians and at least 5 m "
            "from your target.")
    _accepted(CSP_PED, full)
    _refused(CSP_PED, full.split(". Keep")[0] + ".", "quantity")


def test_refuses_a_ruling_after_the_number():
    """'A speed below 4 m/s is not allowed' names what is FORBIDDEN, so the cap
    became a floor; the review found it accepted. The faithful form of the same
    shape - 'speeds above 4 m/s are forbidden' - must still pass."""
    _refused(CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and "
             "20 m. A speed below 4 m/s is not allowed; climb under 2 m/s; turn under "
             "45 deg/s.", "quantity")
    _refused(CSP_DEMO, "Keep out of zone 'nfz-square'. An altitude between 10 m and 20 m "
             "is forbidden. Max speed 4 m/s; climb under 2 m/s; turn under 45 deg/s.",
             "quantity")
    _refused(CSP_DEMO, "Stay out of zone 'nfz-square'. Hold altitude between 10 m and "
             "20 m. Max speed 4 m/s; climb under 2 m/s; turns of 45 deg/s or more are "
             "fine.", "quantity")
    _accepted(CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and "
              "20 m. Speeds above 4 m/s are forbidden; climb under 2 m/s; turn under "
              "45 deg/s.")
    # a negation anywhere in the clause counts, by parity
    _refused(CSP_PED, "The drone should not fly at an altitude no lower than 10 m and no "
             "higher than 20 m. Max speed 4 m/s, climb under 2 m/s, turn under 45 deg/s. "
             "Keep at least 10 m from pedestrians and at least 5 m from your target.",
             "quantity")


def test_kinematic_attributes_must_be_equal_not_merely_included():
    """'Max vertical speed 4 m/s' carried speed AND climb, and a candidate was
    allowed more tags than its source (review, 2026-10-06)."""
    _refused(CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and "
             "20 m. Max vertical speed 4 m/s; climb under 2 m/s; turn under 45 deg/s.",
             "quantity")
    # speed AND climb on one number: a superset of the source's tags, which the
    # subset rule accepted (a mutant restoring it survived every other test)
    _refused(CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and "
             "20 m. Max speed/climb 4 m/s; climb under 2 m/s; turn under 45 deg/s.",
             "quantity")
    # the plain and comparative forms still read as the same attribute
    _accepted(CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and "
              "20 m. Never exceed 4 m/s, climb slower than 2 m/s and turn slower than "
              "45 deg/s.")
    _accepted(CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and "
              "20 m. Max speed 4 m/s; keep your climb speed below 2 m/s; turn under "
              "45 deg/s.")


def test_refuses_conditions_exceptions_and_scope_limits():
    """The stored corridor paraphrase the review flagged began its second half
    with 'Otherwise, remain in corridor ...' - every limit after it became
    conditional, and the validator accepted it."""
    for cand in (
            "Keep out of zone 'nfz-square' unless the operator says otherwise." + REST,
            "Keep out of zone 'nfz-square'. Hold altitude between 10 m and 20 m. Max "
            "speed 4 m/s unless you are behind schedule; climb under 2 m/s; turn under "
            "45 deg/s.",
            "Keep out of zone 'nfz-square'. While climbing, max speed 4 m/s; hold "
            "altitude between 10 m and 20 m; climb under 2 m/s; turn under 45 deg/s.",
            "Keep out of zone 'nfz-square'. Near the zone only, hold altitude between "
            "10 m and 20 m. Max speed 4 m/s; climb under 2 m/s; turn under 45 deg/s.",
            "If you are heading toward it, keep out of zone 'nfz-square'." + REST,
            "Keep out of zone 'nfz-square'. Otherwise, hold altitude between 10 m and "
            "20 m. Max speed 4 m/s; climb under 2 m/s; turn under 45 deg/s."):
        _refused(CSP_DEMO, cand, "modality")
    # a condition that restates the rule's own window is the window
    _accepted(CSP_CORRIDOR, "Please stay out of zone 'nfz-school' entirely while it is in "
              "force, Monday to Friday between 07:30 and 17:30. Fly only inside corridor "
              "'corridor-survey-route', within 20 m of its centerline and between 10 m "
              "and 20 m. Max speed 4 m/s, climb under 2 m/s, turn under 45 deg/s.")
    # coordination is not a condition
    _accepted(CSP_DEMO, "The no-fly zone 'nfz-square' is not somewhere you should ever go. "
              "You need to cruise no lower than 10 m and no higher than 20 m, at a speed "
              "of no more than 4 m/s, while climbing at under 2 m/s and turning at under "
              "45 deg/s.")
    _accepted(TASK_PAD, "travel at 6 m/s until you reach the northeast pad")


def test_refuses_a_hedged_hard_rule():
    """'please' is politeness; 'try to', 'generally' and "you'll want to" make a
    hard limit optional."""
    good = ("Please do not fly into the zone called 'nfz-square'. Keep your altitude "
            "between 10 m and 20 m, and keep your speed to no more than 4 m/s, your "
            "climb rate under 2 m/s and your turn rate under 45 deg/s.")
    _accepted(CSP_DEMO, good)
    _refused(CSP_DEMO, good.replace("Keep your altitude", "Try to keep your altitude"),
             "modality")
    _refused(CSP_DEMO, good.replace("no more than 4", "ideally no more than 4"),
             "modality")
    _refused(CSP_DEMO, good.replace("Keep your altitude", "Generally keep your altitude"),
             "modality")
    _refused(CSP_DEMO, good.replace("Keep your altitude", "You'll want to keep your "
                                                          "altitude"), "modality")


def test_refuses_a_suspended_rule():
    _refused(CSP_DEMO, "Stay out of zone 'nfz-square'." + REST + " These limits are "
             "optional.", "modality")
    _refused(CSP_DEMO, "Stay out of zone 'nfz-square'." + REST + " Ignore every other "
             "limit.", "modality")


def test_refuses_a_dropped_soft_limit_marker():
    """The compiler appends '(soft limit)'. Dropping it turns advice into a hard
    rule, adding it the reverse; both change what the pilot was told."""
    src = "Keep at least 3 m from any building (soft limit)."
    _accepted(src, "Stay at least 3 m clear of every building (soft limit).")
    _refused(src, "Stay at least 3 m clear of every building.", "modality")


def test_refuses_a_paraphrase_far_longer_than_its_source():
    long = "track a red car " + " ".join(["and keep going"] * 10)
    _refused(TASK_CAR, long, "length")


def test_review_probes_that_were_accepted_are_now_refused():
    """The 17 meaning changes the 2026-10-06 review wrote that the validator of
    the day accepted (scratchpad rv_para/adv.py; also in probes.json as the
    review set). Each is refused now, for a reason in its own slot."""
    demo_rest = REST
    cases = [
        (TASK_CAR, "do not follow a red car", "action"),
        (TASK_CAR, "stop following a red car", "action"),
        (TASK_CAR, "never track a red car", "action"),
        (TASK_CAR, "follow the car that is parked next to the red one", "colour"),
        (TASK_CAR, "follow a red car, or any car if you see no red one", "colour"),
        (TASK_FWD, "fly forward and do not avoid restricted areas", "polarity"),
        (TASK_FWD, "fly ahead, and never steer clear of restricted areas", "polarity"),
        (CSP_DEMO, "It is not forbidden to enter zone 'nfz-square'." + demo_rest, "polarity"),
        (CSP_DEMO, "Keep out of zone 'nfz-square' unless the operator says otherwise."
         + demo_rest, "modality"),
        (CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and 20 m. A "
         "speed below 4 m/s is not allowed; climb under 2 m/s; turn under 45 deg/s.",
         "quantity"),
        (CSP_DEMO, "Keep out of zone 'nfz-square'. An altitude between 10 m and 20 m is "
         "forbidden. Max speed 4 m/s; climb under 2 m/s; turn under 45 deg/s.", "quantity"),
        (CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and 20 m. Max "
         "vertical speed 4 m/s; climb under 2 m/s; turn under 45 deg/s.", "quantity"),
        (CSP_DEMO, "Keep out of zone 'nfz-square'. Hold altitude between 10 m and 20 m. "
         "While climbing, max speed 4 m/s; climb under 2 m/s; turn under 45 deg/s.",
         "modality"),
        (CSP_DEMO, "Keep out of zone 'nfz-square'. Near the zone only, hold altitude "
         "between 10 m and 20 m. Max speed 4 m/s; climb under 2 m/s; turn under 45 deg/s.",
         "modality"),
        (CSP_DEMO, "Keep out of zone 'nfz-square'. Generally hold altitude between 10 m and "
         "20 m. Max speed 4 m/s; climb under 2 m/s; turn under 45 deg/s.", "modality"),
        (TASK_PAD, "fly to a spot far away from the northeast pad at 6 m/s", "place"),
        (TASK_FWD, "do not fly forward, and avoid restricted areas", "direction"),
    ]
    assert len(cases) == 17
    for src, cand, code in cases:
        _refused(src, cand, code)


def test_the_stored_corridor_paraphrase_the_review_flagged_is_gone():
    """csp_corridor_survey[6] made the corridor and every limit conditional
    ('Otherwise, remain in corridor ...'). The new validator refuses that text,
    and the store no longer holds it."""
    old = ("Zone 'nfz-school' must never be entered on weekdays between 07:30 and 17:30; "
           "if you notice you are heading toward it in those hours, turn away. Otherwise, "
           "remain in corridor 'corridor-survey-route' at between 10 m and 20 m, no "
           "further than 20 m from its centerline, keeping your speed at or under 4 m/s, "
           "your climb rate under 2 m/s and your turn rate under 45 deg/s.")
    _refused(CSP_CORRIDOR, old, "modality")
    texts = [i["text"] for e in P.load_store(STORE).values() for i in e.paraphrases]
    assert old not in texts
    revised = [i for e in P.load_store(STORE).values() for i in e.paraphrases
               if i.get("revision")]
    assert len(revised) >= 6, "the re-authored items lost their revision notes"


# --------------------------------------------------------------------------- #
# Acceptance: real rewordings the validator must not over-refuse
# --------------------------------------------------------------------------- #

def test_accepts_number_words_and_unit_spellings():
    _accepted(TASK_PAD, "head over to the northeast pad, cruising at six metres "
              "per second")
    _accepted(CSP_DEMO, "Rules for this flight: one, zone 'nfz-square' is a no-go; "
              "two, fly at an altitude between ten and twenty metres; three, speed "
              "four metres per second at most, climb below two metres per second, "
              "turn below forty-five degrees per second.")


def test_accepts_negated_comparatives():
    """'no faster than', 'never exceed', 'no closer than' - the bound is read
    through the negation, not off the comparative alone."""
    _accepted(CSP_PED, "Please keep your altitude between 10 m and 20 m. Please "
              "don't fly faster than 4 m/s, and keep your climb rate under 2 m/s and "
              "your turn rate under 45 deg/s. Please also stay at least 10 m away "
              "from any pedestrian and at least 5 m away from whatever you are "
              "following.")
    _accepted(CSP_PED, "During this flight the drone should hold an altitude no "
              "lower than 10 m and no higher than 20 m. Its speed should never "
              "exceed 4 m/s, its climb rate should stay under 2 m/s, and its turn "
              "rate should stay under 45 deg/s. It should never come closer than "
              "10 m to a pedestrian, nor closer than 5 m to the subject it is "
              "following.")


def test_accepts_faithful_forms_the_review_found_refused():
    """Review, 2026-10-06: of 25 faithful paraphrases, 11 were refused. These
    are the clear cases, fixed in the validator. (Three remain refused on
    purpose or by known limit - see docs/DESIGN-paraphraser.md.)"""
    _accepted(CSP_DEMO, "Zone 'nfz-square' is a no-go area. Keep between 10 and 20 metres "
              "of altitude. Don't exceed 4 m/s of speed, 2 m/s of climb or 45 deg/s of turn.")
    _accepted(CSP_DEMO, "Do not enter the 'nfz-square' zone. Your altitude has to be at least "
              "10 m and at most 20 m. Limit speed to 4 m/s, climb rate to under 2 m/s and "
              "turn rate to under 45 deg/s.")
    _accepted(CSP_PED, "Hold 10-20 m altitude. Keep speed at most 4 m/s, climb under 2 m/s "
              "and turn under 45 deg/s. Never get closer than 10 m to a pedestrian or 5 m "
              "to your target.")
    _accepted(CSP_PED, "Altitude 10 to 20 m; speed up to 4 m/s; climb rate less than 2 m/s; "
              "turn rate less than 45 deg/s. Keep a minimum of 10 m between you and any "
              "pedestrian, and a minimum of 5 m between you and the thing you are following.")


# --------------------------------------------------------------------------- #
# Determinism and ids
# --------------------------------------------------------------------------- #

def test_same_seed_same_paraphrases():
    for backend in ("stored", "template"):
        a = P.paraphrase(CSP_DEMO, 5, seed=1001, backend=backend)
        b = P.paraphrase(CSP_DEMO, 5, seed=1001, backend=backend)
        assert [p.text for p in a] == [p.text for p in b], backend
        assert [p.paraphrase_id for p in a] == [p.paraphrase_id for p in b], backend


def test_seed_changes_the_draw():
    """The other half of determinism: if every seed gave the same draw, a sweep
    over random_seed would re-fly one paraphrase and call it many."""
    for backend in ("stored", "template"):
        draws = {tuple(p.paraphrase_id for p in
                       P.paraphrase(CSP_DEMO, 3, seed=s, backend=backend))
                 for s in range(10)}
        assert len(draws) > 1, f"{backend}: ten seeds, one draw"


def test_full_draw_is_the_whole_set_in_a_seeded_order():
    a = P.paraphrase(TASK_CAR, 8, seed=1, backend="stored")
    b = P.paraphrase(TASK_CAR, 8, seed=2, backend="stored")
    assert sorted(p.index for p in a) == list(range(8))
    assert {p.paraphrase_id for p in a} == {p.paraphrase_id for p in b}


def test_paraphrase_id_is_a_stable_hash_of_source_backend_index_and_text():
    pid = P.make_paraphrase_id(TASK_CAR, "stored", 0, "track a red car")
    # pinned: a change to the hashing scheme re-keys every logged trial. Re-pinned
    # on 2026-10-06 when the text joined the key (no trial was logged before).
    assert pid == PINNED_ID, f"paraphrase_id scheme changed: {pid}"
    assert P.make_paraphrase_id("  follow   a red car ", "stored", 0, "track a red car") == pid
    assert P.make_paraphrase_id(TASK_CAR, "stored", 1, "track a red car") != pid
    assert P.make_paraphrase_id(TASK_CAR, "template-v1", 0, "track a red car") != pid
    assert P.make_paraphrase_id("follow a blue car", "stored", 0, "track a red car") != pid
    # the review's point: a re-authored text is a different wording, so a
    # different id - a per-paraphrase KPI can never merge the two
    assert P.make_paraphrase_id(TASK_CAR, "stored", 0, "chase a red car") != pid


def test_id_does_not_depend_on_seed_or_n():
    one = {p.index: p.paraphrase_id for p in P.paraphrase(TASK_CAR, 2, seed=7)}
    for p in P.paraphrase(TASK_CAR, 8, seed=99):
        if p.index in one:
            assert one[p.index] == p.paraphrase_id
        assert p.paraphrase_id == P.make_paraphrase_id(TASK_CAR, "stored", p.index, p.text)


def test_record_carries_what_a_flight_log_needs():
    p = P.paraphrase(TASK_FWD, 1, seed=3)[0]
    rec = p.to_record()
    for k in ("paraphrase_id", "source_id", "backend", "index", "seed", "text",
              "text_sha256", "source_sha256"):
        assert k in rec, k
    ext = p.manifest_extras()
    assert set(ext) == {"paraphrase_id", "paraphrase_backend", "paraphrase_index",
                        "paraphrase_seed", "paraphrase_source_id",
                        "paraphrase_text_sha256"}, sorted(ext)
    assert ext["paraphrase_id"] == p.paraphrase_id and ext["paraphrase_seed"] == 3
    assert ext["paraphrase_text_sha256"] == P._sha256(p.text)


# --------------------------------------------------------------------------- #
# The stored set: the free-form deliverable
# --------------------------------------------------------------------------- #

def test_every_stored_paraphrase_passes_the_validator():
    store = P.load_store(STORE)
    n = bad = 0
    for entry in store.values():
        for item in entry.paraphrases:
            n += 1
            v = P.validate(entry.source_text, item["text"])
            if not v.ok:
                bad += 1
                print(f"      {entry.source_key}[{item['index']}]: {ascii(v.reasons)}")
    assert n >= 8 * 14, f"only {n} stored paraphrases"
    assert bad == 0, f"{bad}/{n} stored paraphrases fail the validator"


def test_store_sets_are_k8_with_full_provenance():
    for entry in P.load_store(STORE).values():
        assert len(entry.paraphrases) == P.K_DEFAULT, entry.source_key
        prov = entry.provenance
        assert prov["generator"] == "claude-opus-5-5 (agent, 2026-10-06)"
        assert prov["date"] == "2026-10-06"
        # the prompt names the text the set was GENERATED from; a set re-keyed
        # to compile_csp's sentence order says so and must be the same rules
        gen = prov.get("generated_from", entry.source_text)
        assert P.normalise_source(gen) in prov["prompt"], \
            f"{entry.source_key}: the stored prompt does not contain the source"
        if gen != entry.source_text:
            assert entry.kind == "csp_nl" and "rekeyed" in prov, entry.source_key
            assert P.rule_set_key(gen) == P.rule_set_key(entry.source_text), entry.source_key
        texts = [i["text"] for i in entry.paraphrases]
        assert len(set(t.lower() for t in texts)) == len(texts), \
            f"{entry.source_key}: duplicate paraphrases"


def test_stored_sets_are_genuinely_varied():
    """Not near-copies: median word-level edit distance from the source."""
    for entry in P.load_store(STORE).values():
        nov = sorted(P.validate(entry.source_text, i["text"]).novelty
                     for i in entry.paraphrases)
        assert nov[len(nov) // 2] >= 0.40, \
            f"{entry.source_key}: median novelty {nov[len(nov) // 2]:.2f}"


def test_every_instruction_in_scope_has_a_stored_set():
    """Every text a pilot can be shown: each scenario cell's compile_csp
    rendering (with and without its clock), build_prompt's rendering, the city
    policies and the mission texts. Checked against the compiler's CURRENT
    output, so a change to the templated NL makes the stored set stale loudly.

    Until 2026-10-06 this checked build_prompt's rendering only, and the stored
    backend raised UnknownSource for all six CSPs as compile_csp emits them."""
    be = P.StoredBackend(STORE)
    rows = P.default_sources()
    assert any("compile_csp" in r for _, _, o in rows for r in o["renderers"])
    missing = [(k, t) for k, t, _ in rows if not be.has(t)]
    assert not missing, "no stored set for: " + "; ".join(
        f"{k}: {t!r}" for k, t in missing)


def test_compile_csp_and_build_prompt_get_the_same_paraphrases():
    """One rule set, two sentence orders, one set of ids."""
    rows = P.default_sources()
    by_rules = {}
    for _, text, o in rows:
        if o["kind"] == "csp_nl":
            by_rules.setdefault(P.rule_set_key(text), []).append(text)
    pairs = [ts for ts in by_rules.values() if len(ts) > 1]
    assert pairs, "no rule set rendered two ways - nothing to compare"
    for texts in pairs:
        ids = [tuple(p.paraphrase_id for p in P.paraphrase(t, 8, seed=5, backend="stored"))
               for t in texts]
        assert len(set(ids)) == 1, texts


def test_city_follow_objects_still_match_the_launcher():
    """The city task texts are built from run_citylife_follow.ps1's object
    phrases; if the launcher changes, this says so."""
    ps1 = (ROOT / "scripts" / "run_citylife_follow.ps1").read_text(encoding="utf-8")
    for obj in P.CITY_FOLLOW_OBJECTS:
        assert f'"{obj}"' in ps1, f"{obj!r} no longer appears in the launcher"


def _tampered(mutate, expect: str):
    """Copy the store, let `mutate(tmpdir)` damage it, and require load_store
    to refuse with `expect` in the message."""
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        for f in STORE.glob("*.json"):
            shutil.copy(f, tmp / f.name)
        mutate(tmp)
        try:
            P.load_store(tmp)
        except P.StoreIntegrityError as e:
            assert expect in str(e), f"refused, but for {e}"
        else:
            raise AssertionError(f"a tampered store loaded without complaint ({expect})")


def _edit(tmp: Path, name: str, fn):
    f = tmp / name
    doc = json.loads(f.read_text(encoding="utf-8"))
    fn(doc)
    f.write_text(json.dumps(doc, indent=2), encoding="utf-8")


def test_store_detects_a_tampered_text():
    _tampered(lambda t: _edit(t, "task_follow_a_red_car.json",
                              lambda d: d["paraphrases"][2].__setitem__("text",
                                                                        "track a blue car")),
              "text_sha256")


def test_store_detects_a_tampered_id():
    _tampered(lambda t: _edit(t, "task_follow_a_red_car.json",
                              lambda d: d["paraphrases"][2].__setitem__(
                                  "paraphrase_id", "pp-0000000000000000")),
              "paraphrase_id does not recompute")


def test_store_detects_a_source_edited_without_its_hash():
    _tampered(lambda t: _edit(t, "task_follow_a_red_car.json",
                              lambda d: d.__setitem__("source_text", "follow a blue car")),
              "source_sha256")


def test_store_detects_items_out_of_order():
    def swap(d):
        ps = d["paraphrases"]
        ps[0], ps[1] = ps[1], ps[0]
    _tampered(lambda t: _edit(t, "task_follow_a_red_car.json", swap), "out of order")


def test_store_refuses_a_second_set_for_one_source():
    _tampered(lambda t: shutil.copy(t / "task_follow_a_red_car.json", t / "zz_copy.json"),
              "second set for a source")


def test_store_refuses_a_second_set_for_one_rule_set():
    """The same rules in another sentence order are the same set (rule_set_key);
    two files for them would serve two different 'canonical' sets."""
    def add(t: Path):
        e = P.load_store(STORE)
        sim = [x for x in e.values() if x.source_key == "csp_sim_demo_policy"][0]
        import itertools
        reordered = next(" ".join(p) for p in
                         itertools.permutations(P.rule_set_key(sim.source_text))
                         if " ".join(p) != sim.source_text)
        P.write_store(t / "zz_reordered.json", source_key="dup", source_text=reordered,
                      kind="csp_nl", origin={}, provenance={"generator": "test"},
                      paraphrases=[{"text": i["text"]} for i in sim.paraphrases])
    _tampered(add, "second set for the rule set")


def test_store_refuses_an_unknown_schema():
    with tempfile.TemporaryDirectory() as d:
        (Path(d) / "x.json").write_text('{"schema": "something/else"}', encoding="utf-8")
        try:
            P.load_store(d)
        except P.StoreIntegrityError:
            pass
        else:
            raise AssertionError("a file of unknown schema was silently skipped")


def test_write_store_round_trip_and_live_reload():
    """write_store computes ids that load_store recomputes; and the stored
    backend sees a set frozen in the same process (its cache used to keep the
    old store for the life of the process)."""
    with tempfile.TemporaryDirectory() as d:
        be = P.StoredBackend(d)
        assert not be.has(TASK_CAR)
        P.write_store(Path(d) / "s.json", source_key="k", source_text=TASK_CAR, kind="task",
                      origin={}, provenance={"generator": "test"},
                      paraphrases=[{"text": "track a red car", "style": "x"},
                                   {"text": "keep up with a red car"}])
        assert be.has(TASK_CAR)
        e = P.load_store(d)[TASK_CAR]
        assert e.paraphrases[0]["paraphrase_id"] == P.make_paraphrase_id(
            TASK_CAR, "stored", 0, "track a red car")
        out = P.paraphrase(TASK_CAR, 2, seed=0, backend=be)
        assert {p.paraphrase_id for p in out} == {i["paraphrase_id"] for i in e.paraphrases}


# --------------------------------------------------------------------------- #
# Backends: no silent fallback, refusals logged, LLM slot without network
# --------------------------------------------------------------------------- #

class _Capture(logging.Handler):
    def __init__(self):
        super().__init__()
        self.msgs = []

    def emit(self, record):
        self.msgs.append((record.levelno, record.getMessage()))


def _captured(fn):
    h = _Capture()
    lg = logging.getLogger("guardrail.paraphraser")
    lg.addHandler(h)
    try:
        out = fn()
    finally:
        lg.removeHandler(h)
    return out, h.msgs


def test_stored_backend_refuses_an_unknown_source():
    try:
        P.paraphrase("fly to the moon at 6 m/s", 2, seed=0, backend="stored")
    except P.UnknownSource:
        pass
    else:
        raise AssertionError("stored backend served a source it has no set for")


def test_auto_falls_back_to_template_and_says_so_loudly():
    src = "fly to the east pad at 3 m/s"
    out, msgs = _captured(lambda: P.paraphrase(src, 3, seed=0, backend="auto"))
    assert all(p.backend == P.TemplateBackend.name for p in out)
    assert all(p.provenance.get("fallback_from") == "stored" for p in out)
    assert any(lvl == logging.WARNING and "no stored set" in m for lvl, m in msgs), msgs
    stored = P.paraphrase(TASK_CAR, 2, seed=0, backend="auto")
    assert all(p.backend == "stored" for p in stored)


def test_template_backend_output_always_passes_and_never_returns_identity():
    tb = P.TemplateBackend()
    for entry in P.load_store(STORE).values():
        cap = tb.capacity(entry.source_text)
        assert cap >= 2, f"{entry.source_key}: template has {cap} rewordings"
        n = min(8, cap - 1)
        out = P.paraphrase(entry.source_text, n, seed=1001, backend="template")
        assert len(out) == n, entry.source_key
        for p in out:
            assert P.validate(entry.source_text, p.text).ok
            assert p.index > 0, "index 0 is the identity combination"


def test_template_backend_refuses_to_pad_a_small_bank():
    """A one-sentence task has four rewordings; asking for eight is an error,
    not eight draws from four."""
    try:
        P.paraphrase("follow a red car", 8, seed=0, backend="template")
    except P.InsufficientParaphrases:
        pass
    else:
        raise AssertionError("template backend padded or repeated a small bank")


def test_template_carries_a_time_window_onto_every_rewording():
    """The compiler appends "(in force Mon-Fri 07:30-17:30)"; a template that
    dropped it would turn a weekday rule into a permanent one."""
    src = ("Never enter zone 'nfz-school' (in force Mon-Fri 07:30-17:30). Keep "
           "speed at or below 4 m/s, climb rate below 2 m/s and turn rate below "
           "45 deg/s.")
    tb = P.TemplateBackend()
    for i in range(1, tb.capacity(src) + 1):
        text, _ = tb.render(src, i)
        assert "(in force Mon-Fri 07:30-17:30)" in text, text
    _refused(src, "Keep out of zone 'nfz-school'. Cap your speed at 4 m/s, your "
             "climb rate below 2 m/s and your turn rate below 45 deg/s.", "schedule")
    _refused(src, "Keep out of zone 'nfz-school' except Mon-Fri 07:30-17:30. Cap "
             "your speed at 4 m/s, your climb rate below 2 m/s and your turn rate "
             "below 45 deg/s.", "schedule")
    _accepted(src, "On weekdays from 7:30 am to 5:30 pm, zone 'nfz-school' is "
              "off-limits. Cap your speed at 4 m/s, your climb rate below 2 m/s and "
              "your turn rate below 45 deg/s.")


def test_at_or_below_does_not_split_its_clause():
    """Regression, found by the mutation test on 2026-10-06: the "or" inside
    "at or below" was read as a list break, so "Keep speed at or below 4 m/s"
    lost its speed tag - and "Max spacing 4 m/s" passed as its paraphrase."""
    q = [x for x in P.extract_slots(CSP_DEMO).quantities if x.unit == "m/s"][0]
    assert "speed" in q.tags, q
    _refused(CSP_CORRIDOR, "Keep out of zone 'nfz-school' on weekdays from 07:30 to 17:30. "
             "Fly only inside corridor 'corridor-survey-route', no more than 20 m off its "
             "centerline and between 10 m and 20 m up. Max spacing 4 m/s, climb under "
             "2 m/s, turn under 45 deg/s.", "quantity")


CSP_CITY = ("Stay between 6 m and 14 m altitude. Keep speed at or below 5 m/s, "
            "climb rate below 2 m/s and turn rate below 45 deg/s. Keep at least 3 m "
            "from any building. Keep at least 10 m away from any pedestrian. Keep at "
            "least 5 m away from anything you are following.")


def test_regressions_the_mutation_test_found():
    """Four more silent acceptances found by mutating the stored set on
    2026-10-06. Each was a changed slot the validator let through."""
    # an after-number cue not in first position ("of clearance at most")
    _refused(CSP_CITY, "Buildings need 3 m of clearance at most, pedestrians 10 m at "
             "least, and the subject you are following 5 m at least. Speed tops out at "
             "5 m/s; climb rate stays below 2 m/s; turn rate stays below 45 deg/s. "
             "Altitude: no lower than 6 m and no higher than 14 m.", "quantity")
    # a band read from outside ("beyond the 10 m to 20 m band")
    _refused(CSP_DEMO, "Speed limits first: never exceed 4 m/s horizontally, keep any "
             "climb below 2 m/s, and keep turns below 45 deg/s. Altitude must stay "
             "beyond the 10 m to 20 m band. Finally, the zone 'nfz-square' is "
             "strictly off-limits.", "quantity")
    # "a distance no lower than" is a distance, not an altitude
    _refused(CSP_CITY, "For this city mission, the drone has to stay at a distance no "
             "lower than 6 m and no higher than 14 m, must never go faster than 5 m/s, "
             "must climb at less than 2 m/s, and must turn at less than 45 deg/s. On top "
             "of that, it must never get within 3 m of a building or within 10 m of a "
             "pedestrian, nor within 5 m of the thing it is following.", "quantity")
    # an attribute word may not leak across "or" onto the previous limit
    _refused(CSP_CITY, "You'll need to fly somewhere between 6 m and 14 m high without "
             "going over 5 m/s in spacing or climbing faster than 2 m/s or turning "
             "faster than 45 deg/s. Buildings should be given at least 3 m of room, "
             "pedestrians at least 10 m, and the target at least 5 m.", "quantity")


def test_corridor_band_is_an_altitude_band():
    """The compiler's corridor sentence gives its floor and ceiling with no
    attribute word; read beside a stay-inside id they are altitude, and a
    paraphrase that turns them into a distance is refused."""
    bands = [q for q in P.extract_slots(CSP_CORRIDOR).quantities
             if q.unit == "m" and q.bound in ("min", "max") and "lateral" not in q.tags]
    assert bands and all(q.tags == {"altitude"} for q in bands), bands
    _refused(CSP_CORRIDOR, "Keep out of zone 'nfz-school' on weekdays from 07:30 to 17:30. "
             "Fly only inside corridor 'corridor-survey-route', no more than 20 m off its "
             "centerline and between 10 m and 20 m apart. Max speed 4 m/s, climb under "
             "2 m/s, turn under 45 deg/s.", "quantity")


def test_a_distance_along_a_direction_is_an_offset():
    """'120 m ahead' is how far, not how high: tagged 'offset', and a change of
    direction or of the attribute is refused."""
    q = P.extract_slots(TASK_WP).quantities
    assert [x.tags for x in q] == [frozenset({"offset"})], q
    _refused(TASK_WP, "go to the waypoint 120 m behind you, staying at cruise altitude",
             "direction")
    _refused(TASK_WP, "go to the waypoint 120 m up, staying at cruise altitude")


def test_template_bank_cannot_change_without_a_version_bump():
    assert P.template_bank_digest() == PINNED_TEMPLATE_DIGEST, \
        ("the template rule bank changed: bump TEMPLATE_BANK_VERSION (which "
         "re-keys template paraphrase ids) and re-pin the digest here")


def test_bad_arguments_fail_loudly():
    for kwargs in ({"n": 0, "seed": 1}, {"n": 9, "seed": 1},
                   {"n": 2, "seed": None}, {"n": 2, "seed": 1.5},
                   {"n": 2, "seed": True}):
        try:
            P.paraphrase(TASK_CAR, backend="stored", **kwargs)
        except (ValueError, TypeError):
            continue
        raise AssertionError(f"accepted {kwargs}")


class _FakeLLM:
    """Stands in for a service. Returns whatever list it was given."""
    model_id = "fake-llm-0"

    def __init__(self, outs):
        self.outs = outs
        self.prompts = []

    def complete(self, prompt, *, n, seed):
        self.prompts.append((prompt, n, seed))
        return list(self.outs)


def test_llm_slot_makes_no_call_without_a_client():
    try:
        P.paraphrase(TASK_CAR, 1, seed=0, backend=P.LLMServiceBackend())
    except P.BackendUnavailable:
        pass
    else:
        raise AssertionError("LLM backend produced text with no client configured")


def test_llm_refusals_are_logged_with_the_reason():
    fake = _FakeLLM(["track a blue car", "keep up with a red car"])
    refusals = []
    out, msgs = _captured(lambda: P.paraphrase(TASK_CAR, 1, seed=5,
                                               backend=P.LLMServiceBackend(fake),
                                               refusals=refusals))
    assert [p.text for p in out] == ["keep up with a red car"]
    assert out[0].backend == "llm:fake-llm-0"
    assert out[0].provenance["prompt"] == fake.prompts[0][0]
    assert TASK_CAR in fake.prompts[0][0] and fake.prompts[0][2] == 5
    assert len(refusals) == 1 and "colour" in refusals[0]["reasons"][0]
    assert any("track a blue car" in m and "colour" in m for _, m in msgs), msgs


def test_llm_shortfall_is_an_error_not_a_short_list():
    fake = _FakeLLM(["track a blue car"])
    try:
        P.paraphrase(TASK_CAR, 1, seed=0, backend=P.LLMServiceBackend(fake))
    except P.InsufficientParaphrases:
        pass
    else:
        raise AssertionError("returned fewer paraphrases than asked for")


def test_a_repeated_candidate_is_not_two_arms():
    """An LLM that returns one valid text twice must not yield two 'different'
    arms of a per-paraphrase KPI. No test pinned this until the review."""
    fake = _FakeLLM(["keep up with a red car", "keep up with a red car"])
    refusals = []
    try:
        P.paraphrase(TASK_CAR, 2, seed=0, backend=P.LLMServiceBackend(fake),
                     refusals=refusals)
    except P.InsufficientParaphrases:
        pass
    else:
        raise AssertionError("one text was returned as two paraphrases")
    assert refusals and refusals[0]["reasons"][0].startswith("duplicate"), refusals


def test_place_registry_import_failure_is_loud():
    """If the compiler cannot be imported, place slots fall back to a fixed
    list - with a WARNING and PLACES_SOURCE saying so, not silently."""
    saved = (P._places_cache, P.PLACES_SOURCE, sys.modules.get("guardrail.compiler"))
    try:
        P._places_cache = None
        sys.modules["guardrail.compiler"] = None
        names, msgs = _captured(P._places)
        assert names == sorted(P._PLACES_FALLBACK, key=len, reverse=True)
        assert P.PLACES_SOURCE.startswith("fallback"), P.PLACES_SOURCE
        assert any(lvl == logging.WARNING and "could not import" in m for lvl, m in msgs)
    finally:
        P._places_cache, P.PLACES_SOURCE = saved[0], saved[1]
        if saved[2] is not None:
            sys.modules["guardrail.compiler"] = saved[2]
        else:
            sys.modules.pop("guardrail.compiler", None)


# --------------------------------------------------------------------------- #
# AerialVLA's fine-tuned phrases are off limits
# --------------------------------------------------------------------------- #

def test_aerialvla_prompts_are_never_paraphrased():
    """demo/vla_bridge.py: 'A phrase the LoRA never saw in training is worth
    nothing, so the strings are not paraphrased'. The full prompt carries the
    trained frame and the {direction} vocabulary; the service refuses it."""
    sys.path.insert(0, str(ROOT / "demo"))
    import vla_bridge                                            # noqa: E402
    for phrase in sorted(vla_bridge.DIRECTION_VOCABULARY):
        prompt = vla_bridge.build_prompt(phrase, "an orange car")
        for backend in ("auto", "template"):
            try:
                P.paraphrase(prompt, 1, seed=0, backend=backend)
            except P.ProtectedTextError:
                continue
            raise AssertionError(f"paraphrased an AerialVLA prompt: {prompt!r}")


# --------------------------------------------------------------------------- #
# The mutation test, its nulls, the probes, and the report
# --------------------------------------------------------------------------- #

def test_mutants_of_every_stored_paraphrase_are_refused():
    """The validator's recall ON THE GENERATED CLASSES, measured independently
    of how the store was written: every systematic mutation of every stored
    paraphrase. It says nothing about a class mutants() does not generate."""
    rep = _live_report()
    for r in rep["per_source"]:
        for s in r["survivors"][:5]:
            print("      SURVIVOR", r["source_key"], ascii(s))
    assert rep["mutants"] >= 3000, f"only {rep['mutants']} mutants generated"
    for cls in ("negate-verb", "post-prohibit", "exception", "conditional", "hedge",
                "phase", "override", "vertical", "colour-rebind", "widen", "permission",
                "place-role", "negate-modal"):
        assert rep["mutants_by_class"].get(cls, {}).get("generated", 0) > 0, \
            f"the review's mutation class {cls!r} generated nothing"
    assert rep["mutants_refused"] == rep["mutants"], \
        f"{rep['mutants'] - rep['mutants_refused']}/{rep['mutants']} mutants survived"


def test_the_nulls_are_scored_on_both_halves():
    """A null that refuses everything 'catches' every mutant, so each null
    reports how many faithful stored paraphrases it accepts too."""
    rep = _live_report()
    n, m = rep["paraphrases"], rep["mutants"]
    nv = rep["null_validators"]
    assert nv["accept_all_but_exact_copy"] == {**nv["accept_all_but_exact_copy"],
                                               "stored_accepted": n, "mutants_refused": 0}
    # the slot-word bag catches most mutants by refusing most paraphrases too
    bag = nv["slot_word_bag"]
    assert bag["mutants_refused"] < m and bag["stored_accepted"] < n, bag
    assert rep["accepted"] == n and rep["mutants_refused"] == m


def test_probe_rates_are_measured_and_not_zero():
    """The honest number. The review set was used to tune the validator, so
    its 0 false accepts is a regression floor, not an estimate. The held-out
    set was written before the changes and not used to tune them: 5 of its 30
    meaning changes still pass (narrowings no slot sees - 'dark red',
    'pedestrians in the street', 'descend' for 'climb'). Pinned so the design
    doc's numbers cannot drift from the code; if a change moves them, update
    the doc, and say whether the held-out set was looked at."""
    rep = _live_report()
    pr = rep["probes"]
    assert isinstance(pr, dict), pr
    held, rev = pr["heldout_2026_10_06"], pr["review_2026_10_06"]
    assert (held["false_accepts"], held["meaning_changing"]) == (5, 30), held
    assert (held["false_refusals"], held["faithful"]) == (2, 30), held
    assert (rev["false_accepts"], rev["meaning_changing"]) == (0, 28), rev
    assert (rev["false_refusals"], rev["faithful"]) == (3, 30), rev


def test_parser_preview_pins_the_altitude_fallback():
    """The report's 'parse_command reads 6/8 as the same mission' for the
    (40, 40) task: the two misses are 'an altitude of 20', which the compiler's
    regex skips and silently replaces with its 15 m default. Pinned so the
    finding cannot vanish unnoticed - when guardrail/compiler.py is fixed this
    test fails and the doc's preview table must be updated."""
    entry = [e for e in P.load_store(STORE).values() if e.source_key == "task_fly_to_40_40"][0]
    pr = P._parser_reading(entry.source_text, [i["text"] for i in entry.paraphrases])
    assert (pr["same_mission"], pr["n"], pr["same_only_by_default"]) == (6, 8, 0), pr
    misses = [r["text"] for r in pr["rows"] if not r.get("same_mission")]
    assert len(misses) == 2 and all("an altitude of 20" in t for t in misses), misses


def test_validation_report_describes_this_store_and_this_validator():
    """experiments/paraphrases/validation_report.json is quoted in the design
    doc. A report produced from an older store, validator or probe file is a
    number about something else; it must be regenerated, not trusted. Its
    numbers are also recomputed here and compared, not just read back."""
    rep = json.loads((STORE / "validation_report.json").read_text(encoding="utf-8"))
    hint = "re-run: python -m guardrail.paraphraser validate"
    assert rep["store_digest"] == P.store_digest(STORE), "report is stale; " + hint
    assert rep["validator_digest"] == P.validator_digest(), \
        "paraphraser.py changed since the report; " + hint
    live = _live_report()
    assert rep["probes_digest"] == live["probes_digest"], "probes changed; " + hint
    for k in ("sources", "paraphrases", "accepted", "identity_null_refused", "mutants",
              "mutants_refused", "mutants_by_class", "null_validators", "probes",
              "places_source"):
        assert rep[k] == live[k], f"report[{k!r}] != recomputed value; " + hint
    for a, b in zip(rep["per_source"], live["per_source"]):
        assert a.get("parser_reading") == b.get("parser_reading"), a["source_key"]
    assert rep["places_source"] == "guardrail.compiler.PLACES"


PINNED_ID = "pp-ddaa2423665c7006"
PINNED_TEMPLATE_DIGEST = "3226a8c897583410"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
