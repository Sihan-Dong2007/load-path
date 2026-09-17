// Sanity checks for the element stiffness matrix, before it gets used for
// anything else. These are the two checks any FEM tutorial tells you to run
// first, because they catch index-ordering mistakes that are otherwise very
// hard to spot later:
//
//   1. Symmetry: K must equal its own transpose (A's pull on B must equal
//      B's pull on A).
//   2. Rigid-body translation: sliding the whole element sideways, with no
//      stretching at all, must produce ~zero internal force.
import assert from "node:assert/strict";
import { getElementStiffnessMatrix, matVec } from "./web/src/fem.js";
import { assembleGlobalStiffness, solidDensities } from "./web/src/assemble.js";
import { numDofs, nodeId, nodeDofs } from "./web/src/mesh.js";
import { solveDisplacement } from "./web/src/structure.js";
import { computeSensitivities } from "./web/src/sensitivity.js";
import { filterSensitivities } from "./web/src/filter.js";
import { updateDensities } from "./web/src/oc.js";
import { assembleSparseStiffness, sparseMatVec } from "./web/src/sparse.js";
import { conjugateGradient } from "./web/src/cg.js";
import { reduceSystem } from "./web/src/boundary.js";
import { solveLinearSystem } from "./web/src/linalg.js";

const EPSILON = 1e-9;

function assertApproxZero(vector, label) {
  vector.forEach((value, i) => {
    assert.ok(
      Math.abs(value) < EPSILON,
      `${label}[${i}] should be ~0, got ${value}`
    );
  });
}

function assertSymmetric(matrix, label) {
  for (let i = 0; i < matrix.length; i++) {
    for (let j = 0; j < matrix.length; j++) {
      assert.ok(
        Math.abs(matrix[i][j] - matrix[j][i]) < EPSILON,
        `${label}[${i}][${j}]=${matrix[i][j]} should equal ${label}[${j}][${i}]=${matrix[j][i]}`
      );
    }
  }
}

const KE = getElementStiffnessMatrix();

// 1. Symmetry.
assertSymmetric(KE, "KE");
console.log("✓ KE is symmetric");

// 2. Rigid-body translation in x: every node moves +1 in x, 0 in y.
const translateX8 = [1, 0, 1, 0, 1, 0, 1, 0];
assertApproxZero(matVec(KE, translateX8), "KE * translateX");
console.log("✓ rigid-body translation in x produces ~zero force");

// 3. Rigid-body translation in y: every node moves +1 in y, 0 in x.
const translateY8 = [0, 1, 0, 1, 0, 1, 0, 1];
assertApproxZero(matVec(KE, translateY8), "KE * translateY");
console.log("✓ rigid-body translation in y produces ~zero force");

// 4. Assembled meshes must still be symmetric, and rigidly translating the
// *whole* mesh must still produce ~zero force at every node. This is the
// same physical check as on KE alone, now exercising the assembly itself
// (shared nodes between neighboring elements accumulating correctly).
//
// Note: an assembled mesh is NOT expected to equal KE entry-for-entry, even
// for a single element — global node numbering (row-major by node id) walks
// the 4 corners in a different order than the element's own local corner
// order (bottom-left, bottom-right, top-right, top-left), so assembly
// permutes KE's rows/columns rather than reproducing it verbatim. The
// physical checks below don't care about that permutation, which is why
// they're the right thing to assert instead of raw equality.
function assertMeshIsPhysicallySound(numElemX, numElemY) {
  const K = assembleGlobalStiffness(numElemX, numElemY, solidDensities(numElemX, numElemY));
  assertSymmetric(K, `K(${numElemX}x${numElemY})`);

  const n = numDofs(numElemX, numElemY);
  const translateXAll = Array.from({ length: n }, (_, i) => (i % 2 === 0 ? 1 : 0));
  assertApproxZero(matVec(K, translateXAll), `K(${numElemX}x${numElemY}) * translateXAll`);
  const translateYAll = Array.from({ length: n }, (_, i) => (i % 2 === 1 ? 1 : 0));
  assertApproxZero(matVec(K, translateYAll), `K(${numElemX}x${numElemY}) * translateYAll`);
}

assertMeshIsPhysicallySound(1, 1);
assertMeshIsPhysicallySound(3, 2);
console.log("✓ assembled meshes (1x1 and 3x2) are symmetric with zero rigid-body force");

// 5. Solve an actual structure: a mesh pinned at its bottom-left and
// bottom-right corners (like two piers), loaded straight down at the
// top-center node (like a weight dropped in the middle of a bridge deck).
// Because the geometry, supports and load are all left-right symmetric, the
// resulting displacement field is physically required to be symmetric too —
// mirrored nodes must move the same amount vertically and opposite amounts
// horizontally. That's a strong end-to-end check on assembly, boundary
// conditions and the solver all at once.
{
  const numElemX = 4;
  const numElemY = 2;
  const densities = solidDensities(numElemX, numElemY);

  const bottomLeft = nodeId(0, 0, numElemX);
  const bottomRight = nodeId(numElemX, 0, numElemX);
  const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

  const topCenter = nodeId(numElemX / 2, numElemY, numElemX);
  const [, topCenterY] = nodeDofs(topCenter);
  const loads = [[topCenterY, -1]];

  const u = solveDisplacement(numElemX, numElemY, densities, fixedDofs, loads);

  u.forEach((value, i) => {
    assert.ok(Number.isFinite(value), `u[${i}] should be finite, got ${value}`);
  });
  console.log("✓ solved displacement field has no NaN/Infinity");

  assert.ok(u[topCenterY] < 0, "the loaded node should move down (negative y)");
  console.log("✓ the loaded node moves in the direction of the load");

  const MIRROR_EPSILON = 1e-6;
  for (let row = 0; row <= numElemY; row++) {
    for (let col = 0; col <= numElemX; col++) {
      const mirrorCol = numElemX - col;
      const node = nodeId(col, row, numElemX);
      const mirrorNode = nodeId(mirrorCol, row, numElemX);
      const [ux, uy] = nodeDofs(node).map((d) => u[d]);
      const [mux, muy] = nodeDofs(mirrorNode).map((d) => u[d]);

      assert.ok(
        Math.abs(uy - muy) < MIRROR_EPSILON,
        `node(${col},${row}) uy=${uy} should match mirror node(${mirrorCol},${row}) uy=${muy}`
      );
      assert.ok(
        Math.abs(ux + mux) < MIRROR_EPSILON,
        `node(${col},${row}) ux=${ux} should be -1x mirror node(${mirrorCol},${row}) ux=${mux}`
      );
    }
  }
  console.log("✓ the displacement field is left-right symmetric, as the setup requires");
}

// 6. Sensitivities on that same symmetric bridge setup: every sensitivity
// must be <= 0 (removing material never helps compliance), and — because
// the setup is left-right symmetric — mirrored elements must get the same
// sensitivity.
{
  const numElemX = 4;
  const numElemY = 2;
  const densities = solidDensities(numElemX, numElemY);

  const bottomLeft = nodeId(0, 0, numElemX);
  const bottomRight = nodeId(numElemX, 0, numElemX);
  const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

  const topCenter = nodeId(numElemX / 2, numElemY, numElemX);
  const [, topCenterY] = nodeDofs(topCenter);
  const loads = [[topCenterY, -1]];

  const u = solveDisplacement(numElemX, numElemY, densities, fixedDofs, loads);
  const dc = computeSensitivities(numElemX, numElemY, densities, u);

  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      assert.ok(
        dc[ely][elx] <= EPSILON,
        `dc[${ely}][${elx}]=${dc[ely][elx]} should be <= 0`
      );
    }
  }
  console.log("✓ every sensitivity is <= 0");

  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      const mirrorElx = numElemX - 1 - elx;
      assert.ok(
        Math.abs(dc[ely][elx] - dc[ely][mirrorElx]) < 1e-6,
        `dc[${ely}][${elx}]=${dc[ely][elx]} should match mirror dc[${ely}][${mirrorElx}]=${dc[ely][mirrorElx]}`
      );
    }
  }
  console.log("✓ sensitivities are left-right symmetric");
}

// 7. Filtering: a uniform field should pass through unchanged (there's
// nothing to smooth toward), while a single spike in an otherwise-zero
// field should get smaller at the spike and spread to its neighbors —
// otherwise the filter isn't actually averaging with neighbors.
{
  const numElemX = 5;
  const numElemY = 5;
  const uniformDensities = solidDensities(numElemX, numElemY);
  const uniformSensitivities = Array.from({ length: numElemY }, () => new Array(numElemX).fill(-5));

  const filteredUniform = filterSensitivities(numElemX, numElemY, uniformDensities, uniformSensitivities, 1.5);
  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      assert.ok(
        Math.abs(filteredUniform[ely][elx] - -5) < EPSILON,
        `filtered uniform field at [${ely}][${elx}] should stay -5, got ${filteredUniform[ely][elx]}`
      );
    }
  }
  console.log("✓ filtering a uniform field leaves it unchanged");

  const spike = Array.from({ length: numElemY }, () => new Array(numElemX).fill(0));
  spike[2][2] = -100;
  const filteredSpike = filterSensitivities(numElemX, numElemY, uniformDensities, spike, 1.5);

  assert.ok(
    Math.abs(filteredSpike[2][2]) < 100,
    `filtered spike at the center should be smaller in magnitude than the raw -100, got ${filteredSpike[2][2]}`
  );
  assert.ok(
    filteredSpike[2][1] < 0 && filteredSpike[1][2] < 0,
    "the spike should spread some (negative) sensitivity to its immediate neighbors"
  );
  assert.ok(
    Math.abs(filteredSpike[0][0]) < EPSILON,
    "a corner far from the spike should be unaffected"
  );
  console.log("✓ filtering spreads a spike to its neighbors instead of passing it through untouched");
}

// 8. OC density update: starting from a uniform design already at the
// target volume fraction, a uniform sensitivity field should leave it
// uniform (nothing distinguishes one element from another). A non-uniform
// sensitivity field should push more material toward the elements with the
// larger-magnitude (more negative) sensitivity, while still respecting the
// overall volume constraint.
{
  const numElemX = 4;
  const numElemY = 4;
  const volumeFraction = 0.5;
  const startDensities = Array.from({ length: numElemY }, () => new Array(numElemX).fill(volumeFraction));

  const uniformDc = Array.from({ length: numElemY }, () => new Array(numElemX).fill(-1));
  const uniformResult = updateDensities(numElemX, numElemY, startDensities, uniformDc, volumeFraction);
  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      assert.ok(
        Math.abs(uniformResult[ely][elx] - volumeFraction) < 1e-3,
        `uniform update at [${ely}][${elx}] should stay near ${volumeFraction}, got ${uniformResult[ely][elx]}`
      );
    }
  }
  console.log("✓ a uniform sensitivity field keeps a uniform design uniform, at the target volume");

  // Left half is much more "important" (larger-magnitude sensitivity) than
  // the right half.
  const skewedDc = Array.from({ length: numElemY }, (_, ely) =>
    Array.from({ length: numElemX }, (_, elx) => (elx < numElemX / 2 ? -10 : -0.1))
  );
  const skewedResult = updateDensities(numElemX, numElemY, startDensities, skewedDc, volumeFraction);

  for (let ely = 0; ely < numElemY; ely++) {
    assert.ok(
      skewedResult[ely][0] > skewedResult[ely][numElemX - 1],
      `row ${ely}: the more-important left element (${skewedResult[ely][0]}) should end up denser than the less-important right element (${skewedResult[ely][numElemX - 1]})`
    );
  }
  const skewedVolume = skewedResult.flat().reduce((sum, x) => sum + x, 0);
  const targetVolume = volumeFraction * numElemX * numElemY;
  assert.ok(
    Math.abs(skewedVolume - targetVolume) < 0.5,
    `total volume ${skewedVolume} should stay near the target ${targetVolume}`
  );
  console.log("✓ a skewed sensitivity field shifts material toward the more important elements, near the target volume");
}

// 9. Sparse assembly should be a pure storage change, not a math change:
// for the same densities, it must match dense assembly entry-for-entry,
// and it must pass the exact same rigid-body physical check as the dense
// version.
function sparseToDense(K, n) {
  return K.map((row) => {
    const dense = new Array(n).fill(0);
    for (const [col, value] of row) dense[col] = value;
    return dense;
  });
}

function assertSparseMatchesDense(numElemX, numElemY) {
  const densities = solidDensities(numElemX, numElemY);
  const dense = assembleGlobalStiffness(numElemX, numElemY, densities);
  const sparse = assembleSparseStiffness(numElemX, numElemY, densities);
  const n = numDofs(numElemX, numElemY);
  const sparseDense = sparseToDense(sparse, n);

  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      assert.ok(
        Math.abs(dense[i][j] - sparseDense[i][j]) < EPSILON,
        `sparse/dense mismatch at [${i}][${j}]: dense=${dense[i][j]} sparse=${sparseDense[i][j]}`
      );
    }
  }
}

assertSparseMatchesDense(1, 1);
assertSparseMatchesDense(3, 2);
console.log("✓ sparse assembly matches dense assembly exactly");

function assertSparseIsPhysicallySound(numElemX, numElemY) {
  const K = assembleSparseStiffness(numElemX, numElemY, solidDensities(numElemX, numElemY));
  const n = numDofs(numElemX, numElemY);
  const translateXAll = Array.from({ length: n }, (_, i) => (i % 2 === 0 ? 1 : 0));
  assertApproxZero(sparseMatVec(K, translateXAll), `sparse K(${numElemX}x${numElemY}) * translateXAll`);
  const translateYAll = Array.from({ length: n }, (_, i) => (i % 2 === 1 ? 1 : 0));
  assertApproxZero(sparseMatVec(K, translateYAll), `sparse K(${numElemX}x${numElemY}) * translateYAll`);
}

assertSparseIsPhysicallySound(1, 1);
assertSparseIsPhysicallySound(3, 2);
console.log("✓ sparse assembly passes the same rigid-body check as dense");

// 10. CG on a tiny hand-checkable system: 4x+y=1, x+3y=2 solves to
// x=1/11, y=7/11 (substitute the first equation into the second to check).
{
  const A = [
    [4, 1],
    [1, 3],
  ];
  const b = [1, 2];
  const { x } = conjugateGradient((v) => matVec(A, v), b);

  assert.ok(Math.abs(x[0] - 1 / 11) < 1e-6, `x[0]=${x[0]} should be ~${1 / 11}`);
  assert.ok(Math.abs(x[1] - 7 / 11) < 1e-6, `x[1]=${x[1]} should be ~${7 / 11}`);
  console.log("✓ CG matches the hand-solved answer for a tiny system");
}

// 11. CG must agree with the dense solver on an actual structural
// problem — the same symmetric bridge setup from check 5 — not just on a
// textbook toy example.
{
  const numElemX = 4;
  const numElemY = 2;
  const densities = solidDensities(numElemX, numElemY);

  const bottomLeft = nodeId(0, 0, numElemX);
  const bottomRight = nodeId(numElemX, 0, numElemX);
  const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

  const topCenter = nodeId(numElemX / 2, numElemY, numElemX);
  const [, topCenterY] = nodeDofs(topCenter);
  const n = numDofs(numElemX, numElemY);
  const f = new Array(n).fill(0);
  f[topCenterY] = -1;

  const K = assembleGlobalStiffness(numElemX, numElemY, densities);
  const { Kff, ff } = reduceSystem(K, f, fixedDofs);

  const denseSolution = solveLinearSystem(Kff, ff);
  const { x: cgSolution, iterations } = conjugateGradient((v) => matVec(Kff, v), ff);

  denseSolution.forEach((value, i) => {
    assert.ok(
      Math.abs(value - cgSolution[i]) < 1e-5,
      `dense[${i}]=${value} should match CG[${i}]=${cgSolution[i]}`
    );
  });
  console.log(`✓ CG matches the dense solver on the actual bridge problem (${iterations} iterations)`);
}

console.log("\nAll checks passed.");
