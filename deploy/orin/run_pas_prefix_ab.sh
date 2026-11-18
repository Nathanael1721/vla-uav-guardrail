#!/usr/bin/env bash
# The prefix A/B on the hil follow (WP4-15; guardrail/vla_prefix.py): N pairs
# of the SAME follow - one scene, one policy bundle, one object phrase, one
# seed - flown with the Prefix Compiler's CSP in the VLA's prompt and without
# it, each flight through deploy/orin/run_pas_perception.sh on the AGX Orin.
#
#   CAMPAIGN=pab_aerialvla_01 FAMILY=aerialvla G1_VERDICT=~/g1_verdict.json \
#       N_PAIRS=3 bash deploy/orin/run_pas_prefix_ab.sh [node args...]
#
# What it flies, per pair k = 1..N_PAIRS:
#   TAG=<CAMPAIGN>_p<k>_<arm>, AB_PAIR=<CAMPAIGN>_p<k>, PREFIX=<arm>, and the
#   node's --seed SEED (default 20261006, the node's own default) on every
#   flight, so the two arms of a pair differ in the prompt alone.
#   ARMS (default "csp off") is flown in ABBA order: pair 1 in the order given,
#   pair 2 reversed, and so on, so neither arm always flies first on a scene
#   that has run longer. The first pair starts with the prefix arm: a prefix
#   the model's slot cannot take (its P0 rules over the budget, never
#   truncated) is refused before take-off, and no flight is spent first.
#   ARMS="csp off neutral" adds the length-matched rule-free arm (a third
#   flight per pair, rotated the same way). It is matched in the model's own
#   tokens: on AerialVLA with OpenVLA's tokenizer.json under OPENVLA_DIR
#   (default ~/models/openvla-7b, the AGX's base model; checked before the
#   first flight), never with the budget's upper-bound counter.
#   AA=1 adds the A/A repeat: the csp arm flown a second time in each pair
#   (TAG <CAMPAIGN>_p<k>_csp_aa, rotated with the others), the noise floor the
#   report reads the delta against.
#
# What the A/B measures: the models were never trained with rule text in the
# prompt, so the delta is what the prompt change does, not an expected effect.
# AIoT-VLA (FAMILY=aiotvla, the default) has 11 CLIP tokens beside "a red car"
# in OWL-ViT's 16-token text window; the follow policy's P0 rules need 45
# (alt-band 10, bld-clearance 9, standoff-any 12, standoff-pedestrian 14), so
# its csp arm is refused (vla_prefix_refused.json in the run folder, each
# rule's count in it) and this script stops there. AerialVLA
# (FAMILY=aerialvla) reads the CSP on its prompt's instruction line
# (256-token budget) and is the one model that can fly the A/B. It lost the
# car on every C2 follow: if it does so on both arms, mission success is 0
# against 0 and the report says so; read repairs per episode, repair
# magnitude and the follow's share within 30 m against the A/A floor (AA=1).
# The seed fixes the node's random generators, not the scene's traffic.
#
# The report: tools/kpi_report.py --runs 'demo/out/<CAMPAIGN>_p*' --out <stem>
# (section "A/B: prefix on / off", "Flown by a VLA that reads the prefix").
#
# Environment: everything run_pas_perception.sh reads (G1_VERDICT, FAMILY,
# SECONDS_, BUNDLE, VLA_PORT, ...) passes through, except TAG, PREFIX and
# AB_PAIR, which this script sets. Also:
#   CAMPAIGN   the run-folder prefix (required; a plain name)
#   N_PAIRS    pairs to fly (default 3)
#   ARMS       the arms of a pair (default "csp off")
#   AA         1 = also fly the csp arm's A/A repeat in each pair (default 0)
#   SEED       the node's --seed for every flight (default 20261006)
#   PAUSE_S    seconds between flights (default 0)
#   OPENVLA_DIR  OpenVLA's folder holding tokenizer.json, for AerialVLA's
#              neutral arm (default ~/models/openvla-7b; exported to the node)
# VLAGUARD_DRY_RUN=1 prints each flight's command (run_pas_perception.sh's
# dry run) and flies nothing (tests/test_deploy_configs.py).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

die() { echo "*** $2" >&2; exit "$1"; }

: "${CAMPAIGN:?set CAMPAIGN to the run-folder prefix (e.g. pab_aerialvla_01)}"
case "$CAMPAIGN" in
  ""|*/*|.*|*" "*) die 64 "CAMPAIGN '$CAMPAIGN' must be a plain directory name" ;;
esac
N_PAIRS="${N_PAIRS:-3}"
case "$N_PAIRS" in
  ''|*[!0-9]*|0) die 64 "N_PAIRS '$N_PAIRS' is not a positive whole number" ;;
esac
SEED="${SEED:-20261006}"
case "$SEED" in
  ''|*[!0-9]*) die 64 "SEED '$SEED' is not a whole number" ;;
esac
PAUSE_S="${PAUSE_S:-0}"
read -r -a arms <<< "${ARMS:-csp off}"
[ "${#arms[@]}" -ge 2 ] || die 64 "ARMS '${ARMS:-}' needs two arms or more (csp off [neutral])"
seen=" "
has_off=0
for a in "${arms[@]}"; do
  case "$a" in
    off|csp|neutral) ;;
    *) die 64 "ARMS: '$a' is none of off csp neutral" ;;
  esac
  case "$seen" in *" $a "*) die 64 "ARMS: '$a' given twice" ;; esac
  seen="$seen$a "
  [ "$a" = off ] && has_off=1
done
[ "$has_off" = 1 ] || die 64 "ARMS '${ARMS:-}' has no off arm: an A/B needs it"
for a in "$@"; do
  case "$a" in
    --seed*|--vla-prefix*|--ab-pair*|--tag*)
      die 64 "$a: this script sets it (SEED, ARMS, CAMPAIGN)" ;;
  esac
done
case "${FAMILY:-aiotvla}" in
  servo|estimator) die 64 "FAMILY=$FAMILY flies no VLA: there is no prompt for a prefix" ;;
esac
case "${AA:-0}" in
  0) ;;
  1)
    case "$seen" in
      *" csp "*) arms+=(csp_aa) ;;
      *) die 64 "AA=1 repeats the csp arm, and ARMS '${ARMS:-}' has none" ;;
    esac ;;
  *) die 64 "AA '$AA' is neither 0 nor 1" ;;
esac
# AerialVLA's neutral arm is length-matched with OpenVLA's own tokenizer
# (guardrail/vla_prefix.py); the node refuses the arm without it, so say so
# before the first flight rather than after a CSP pair.
case "$seen" in
  *" neutral "*)
    if [ "${FAMILY:-aiotvla}" = aerialvla ]; then
      export OPENVLA_DIR="${OPENVLA_DIR:-$HOME/models/openvla-7b}"
      echo "==> neutral arm: length-matched with $OPENVLA_DIR/tokenizer.json"
      if [ "${VLAGUARD_DRY_RUN:-0}" != "1" ] && [ ! -f "$OPENVLA_DIR/tokenizer.json" ]; then
        die 66 "no $OPENVLA_DIR/tokenizer.json: the neutral arm cannot be length-matched in OpenVLA's tokens (set OPENVLA_DIR, or drop neutral from ARMS)"
      fi
    fi ;;
esac

n_arms="${#arms[@]}"
for ((k = 1; k <= N_PAIRS; k++)); do
  order=("${arms[@]}")
  if (( k % 2 == 0 )); then
    order=()
    for ((i = n_arms - 1; i >= 0; i--)); do order+=("${arms[$i]}"); done
  fi
  for arm in "${order[@]}"; do
    tag="${CAMPAIGN}_p${k}_${arm}"
    prefix="${arm%_aa}"     # csp_aa: the csp arm again, the pair's A/A repeat
    echo "==> pair $k/$N_PAIRS, arm $arm: TAG=$tag AB_PAIR=${CAMPAIGN}_p${k} --seed $SEED"
    TAG="$tag" PREFIX="$prefix" AB_PAIR="${CAMPAIGN}_p${k}" \
      bash "$ROOT/deploy/orin/run_pas_perception.sh" --seed "$SEED" "$@" \
      || die $? "pair $k arm $arm ended with an error: stopped (demo/out/$tag; a refused prefix writes vla_prefix_refused.json there)"
    if [ "${VLAGUARD_DRY_RUN:-0}" != "1" ] && [ "$PAUSE_S" != "0" ]; then
      sleep "$PAUSE_S"
    fi
  done
done
echo "==> $N_PAIRS pair(s) flown: tools/kpi_report.py --runs 'demo/out/${CAMPAIGN}_p*' --out <stem>"
