#!/bin/zsh
# End-to-end validation: predictor (variant B, mode 0) vs stock (variant A)
set -e
WT=/Users/yuval/git/mujoco/.claude/worktrees/house-of-cards
BIN=$WT/build/bin/cone_bench
cd $WT

echo "=== hoc stable ==="
$BIN model/cards/house_of_cards.xml 1000 0 3 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== hoc tilt2 (collapse) ==="
$BIN model/cards/house_of_cards.xml 1000 0 3 -tilt 2 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== arch ==="
$BIN model/arch/hyperbolic.xml 1000 0 3 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== clump t1 ==="
$BIN bench/clump.xml 400 0 3 -tilt 1 -settle 800 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== clump t5 ==="
$BIN bench/clump.xml 400 0 3 -tilt 5 -settle 800 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== free t5 ==="
$BIN bench/clump_free.xml 400 0 3 -tilt 5 -settle 800 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== clump t5 noisland ==="
$BIN bench/clump.xml 300 0 3 -tilt 5 -settle 800 -noisland 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== free t5 condim4 ==="
$BIN bench/clump_free.xml 300 0 3 -tilt 5 -condim 4 -settle 800 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== free t5 condim6 ==="
$BIN bench/clump_free.xml 300 0 3 -tilt 5 -condim 6 -settle 800 2>/dev/null | grep -E "all steps|worst|equivalence"
echo "=== clump t5 nowarmstart ==="
$BIN bench/clump.xml 300 0 3 -tilt 5 -settle 800 -nowarmstart 2>/dev/null | grep -E "all steps|worst|equivalence"
