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
#   flight per pair, rotated the same way).
#
# What the A/B measures: the models were never trained with rule text in the
# prompt, so the delta is what the prompt change does, not an expected effect.
# AIoT-VLA (FAMILY=aiotvla, the default) has 11 CLIP tokens beside "a red car"
# in OWL-ViT's 16-token text window; the follow policy's P0 rules need 45, so
# its csp arm is refused (vla_prefix_refused.json in the run folder) and this
# script stops there. AerialVLA (FAMILY=aerialvla) reads the CSP on its
# prompt's instruction line (256-token budget) and can fly the A/B.
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
#   SEED       the node's --seed for every flight (default 20261006)
#   PAUSE_S    seconds between flights (default 0)
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

n_arms="${#arms[@]}"
for ((k = 1; k <= N_PAIRS; k++)); do
  order=("${arms[@]}")
  if (( k % 2 == 0 )); then
    order=()
    for ((i = n_arms - 1; i >= 0; i--)); do order+=("${arms[$i]}"); done
  fi
  for arm in "${order[@]}"; do
    tag="${CAMPAIGN}_p${k}_${arm}"
    echo "==> pair $k/$N_PAIRS, arm $arm: TAG=$tag AB_PAIR=${CAMPAIGN}_p${k} --seed $SEED"
    TAG="$tag" PREFIX="$arm" AB_PAIR="${CAMPAIGN}_p${k}" \
      bash "$ROOT/deploy/orin/run_pas_perception.sh" --seed "$SEED" "$@" \
      || die $? "pair $k arm $arm ended with an error: stopped (demo/out/$tag; a refused prefix writes vla_prefix_refused.json there)"
    if [ "${VLAGUARD_DRY_RUN:-0}" != "1" ] && [ "$PAUSE_S" != "0" ]; then
      sleep "$PAUSE_S"
    fi
  done
done
echo "==> $N_PAIRS pair(s) flown: tools/kpi_report.py --runs 'demo/out/${CAMPAIGN}_p*' --out <stem>"
