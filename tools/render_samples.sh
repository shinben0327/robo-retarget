#!/usr/bin/env bash
# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
# Retarget and render the shipped sample motions across every ordered pair of
# {g1, k1, t1, t2}: each robot is the target of three clips (one from each other
# robot) and the source of three; H1 is source and target twice, OP3 and Pi+ (no
# motions of their own) are targets twice, and H2, AI Sapiens and Wolfgang once.
# A sixth field passes extra options for that clip (the 60 fps G1 clips need --fps 60).
# Outputs: outputs/samples/<src>_to_<tgt>/
#
#   tools/render_samples.sh                  # all 23 pairs
#   tools/render_samples.sh g1_to_k1 --hq    # pairs matching a pattern; extra args go to robo-retarget
set -euo pipefail
cd "$(dirname "$0")/.."
M=assets/motions
FILTER=${1:-}; shift || true

# src tgt motion start frames  (G1 clips are 30 fps, Booster clips 50 fps: ~20 s / ~15 s)
SAMPLES=(
  "g1 k1 $M/g1/dance1_subject2.csv 300 600"
  "g1 t1 $M/g1/fight1_subject2.csv 300 600"
  "g1 t2 $M/g1/jumps1_subject1.csv 300 600"
  "k1 g1 $M/k1/k1_mj2_seg1.csv 0 750"
  "k1 t1 $M/k1/k1_fight_001.csv 0 750"
  "k1 t2 $M/k1/k1_mj2_seg1.csv 500 750"
  "t1 g1 $M/t1/t1_dance.csv 0 750"
  "t1 k1 $M/t1/t1_dance.csv 750 750"
  "t1 t2 $M/t1/t1_dance.csv 400 750"
  "t2 g1 $M/t2/t2_dance.csv 0 750"
  "t2 k1 $M/t2/t2_dance.csv 400 750"
  "t2 t1 $M/t2/t2_dance.csv 200 750"
  "g1 op3 $M/g1/walk1_subject1.csv 300 600"
  "k1 op3 $M/k1/k1_fight_001.csv 0 750"
  "g1 h1 $M/g1/dance1_subject2.csv 300 600"
  "k1 h1 $M/k1/k1_fight_001.csv 0 750"
  "h1 k1 $M/h1/walk1_subject1.csv 300 600"
  "h1 t1 $M/h1/walk1_subject1.csv 900 600"
  "g1 pi_plus $M/g1/dance1_subject2.csv 300 600"
  "g1 h2 $M/g1/dance_102.csv 180 900 --fps 60"
  "g1 ai_sapiens $M/g1/dance1_subject2.csv 300 600"
  "k1 wolfgang $M/k1/k1_fight_001.csv 0 750"
  "k1 pi_plus $M/k1/k1_fight_001.csv 0 750"
)

for s in "${SAMPLES[@]}"; do
  read -r src tgt motion start frames extra <<<"$s"
  [[ -n "$FILTER" && "${src}_to_${tgt}" != *"$FILTER"* ]] && continue
  # shellcheck disable=SC2086  # $extra holds options to split
  uv run robo-retarget --src "$src" --tgt "$tgt" --motion "$motion" --start "$start" --frames "$frames" \
    ${extra:-} --out "outputs/samples/${src}_to_${tgt}/$(basename "$motion" .csv)_f${start}.pkl" --render "$@"
done
