#!/usr/bin/env bash
# Smoke-test for the headless --measure-path/--measure-loop CLI, run in CI
# on every push/PR across Linux/Windows/macOS. Checks against ANALYTIC
# ground truth, not just "the binary ran without crashing":
#
#   flat_grid.obj: a flat 21x21 grid plane where geodesic distance equals
#   Euclidean distance by construction. Vertex 0 = (0,0,0), vertex 125 =
#   (20,5,0) -> expected geodesic length sqrt(20^2+5^2) = 20.615528...
#
#   cylinder.obj: radius-3 solid cylinder. The 64-vertex rim loop's
#   DELAUNAY_LENGTH must stay within 1% of the analytic circumference
#   2*pi*3 = 18.849556..., since Delaunay refinement is a no-op on length
#   (see src/main.cpp's module comment on runHeadlessMeasure for why
#   --measure-loop does NOT call iterativeShorten, unlike --measure-path).
set -euo pipefail
BIN="${1:?usage: smoke_test.sh path/to/flip_geodesics}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== smoke test 1/2: open path, flat grid (analytic geodesic = Euclidean) ==="
OUT1=$("$BIN" "$HERE/flat_grid.obj" --measure-path 0,125)
echo "$OUT1"
GEO1=$(echo "$OUT1" | awk '/^GEODESIC_LENGTH/ {print $2}')
python3 -c "
import sys
got = float('$GEO1')
expected = 20.615528128088304
err = abs(got - expected)
print(f'got={got} expected={expected} abs_err={err}')
assert err < 0.001, f'FAIL: open-path geodesic length off by {err} (expected < 0.001)'
print('PASS')
"

echo "=== smoke test 2/2: closed loop, cylinder rim (analytic circumference) ==="
LOOP="69,71,73,75,77,79,81,83,85,87,89,91,93,95,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,127,129,2,5,7,9,11,13,15,17,19,21,23,25,27,29,31,33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,67"
OUT2=$("$BIN" "$HERE/cylinder.obj" --measure-loop "$LOOP")
echo "$OUT2"
DEL2=$(echo "$OUT2" | awk '/^DELAUNAY_LENGTH/ {print $2}')
python3 -c "
import math
got = float('$DEL2')
expected = 2 * math.pi * 3.0
err = abs(got - expected) / expected
print(f'got={got} expected={expected} rel_err={err:.4f}')
assert err < 0.01, f'FAIL: loop DELAUNAY_LENGTH off by {err*100:.2f}% (expected < 1%)'
print('PASS')
"

echo "=== all smoke tests passed ==="
