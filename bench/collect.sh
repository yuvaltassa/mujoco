#!/bin/zsh
# Data collection for the cone-fold predictor: rollouts with in-engine logging.
set -e
WT=/Users/yuval/git/mujoco/.claude/worktrees/house-of-cards
SP=/private/tmp/claude-501/-Users-yuval-git-mujoco/d6e61d91-ca96-4813-bab6-ba8bad1a4f85/scratchpad/conelogs
BIN=$WT/build/bin/cone_bench
cd $WT
rm -f $SP/*.csv 2>/dev/null || true

MJ_CONE_LOG=$SP/hoc_stable.csv   $BIN model/cards/house_of_cards.xml 1500 0 1 -rollout 2>/dev/null
MJ_CONE_LOG=$SP/hoc_tilt2.csv    $BIN model/cards/house_of_cards.xml 1500 0 1 -rollout -tilt 2 2>/dev/null
MJ_CONE_LOG=$SP/arch.csv         $BIN model/arch/hyperbolic.xml 1500 0 1 -rollout 2>/dev/null
MJ_CONE_LOG=$SP/clump_t1.csv     $BIN bench/clump.xml 600 0 1 -rollout -tilt 1 -settle 800 2>/dev/null
MJ_CONE_LOG=$SP/clump_t5.csv     $BIN bench/clump.xml 600 0 1 -rollout -tilt 5 -settle 800 2>/dev/null
MJ_CONE_LOG=$SP/free_t5.csv      $BIN bench/clump_free.xml 600 0 1 -rollout -tilt 5 -settle 800 2>/dev/null
MJ_CONE_LOG=$SP/clump_t5_noisl.csv $BIN bench/clump.xml 400 0 1 -rollout -tilt 5 -settle 800 -noisland 2>/dev/null
MJ_CONE_LOG=$SP/free_t5_cd6.csv  $BIN bench/clump_free.xml 400 0 1 -rollout -tilt 5 -condim 6 -settle 800 2>/dev/null
MJ_CONE_LOG=$SP/free_t5_cd4.csv  $BIN bench/clump_free.xml 400 0 1 -rollout -tilt 5 -condim 4 -settle 800 2>/dev/null
MJ_CONE_LOG=$SP/clump_t5_nows.csv $BIN bench/clump.xml 400 0 1 -rollout -tilt 5 -settle 800 -nowarmstart 2>/dev/null

wc -l $SP/*.csv
