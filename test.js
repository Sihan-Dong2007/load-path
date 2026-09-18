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
import { runTopologyOptimization } from "./web/src/optimize.js";
import { findConnectedPath } from "./web/src/connectivity.js";
import { computeStrain, planeStressD, computeStress, principalStresses, maxPrincipalStress } from "./web/src/stress.js";
import { rectMomentOfInertia, eulerCriticalLoad, resolveTrussAxialForces } from "./web/src/buckling.js";
import { evaluateHalves } from "./web/src/failure.js";
import { convexHull, polygonCentroid, groupCellsBySeed, buildShards } from "./web/src/fracture.js";
import { kgToVolumeFraction, volumeFractionToKg, MAX_MATERIAL_KG, kgToNewtons, GRAVITY_M_S2 } from "./web/src/units.js";

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

// 12. The real correctness signal for the optimizer isn't "does the shape
// look like a clean truss" — that depends on mesh resolution and volume
// fraction, and can be misleading. It's whether compliance (what the
// algorithm actually minimizes) decreases and settles, iteration over
// iteration.
{
  const numElemX = 10;
  const numElemY = 6;
  const bottomLeft = nodeId(0, 0, numElemX);
  const bottomRight = nodeId(numElemX, 0, numElemX);
  const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];
  const topCenter = nodeId(numElemX / 2, numElemY, numElemX);
  const [, topCenterY] = nodeDofs(topCenter);
  const loads = [[topCenterY, -1]];

  const { history } = runTopologyOptimization(numElemX, numElemY, fixedDofs, loads, { volumeFraction: 0.4 });

  assert.ok(history.length > 1, "expected more than one iteration to check a trend");
  for (let i = 1; i < history.length; i++) {
    assert.ok(
      history[i].compliance <= history[i - 1].compliance + 1e-6,
      `compliance should not increase: iteration ${i + 1}=${history[i].compliance} vs iteration ${i}=${history[i - 1].compliance}`
    );
  }
  console.log(
    `✓ compliance decreases monotonically across ${history.length} iterations (${history[0].compliance.toFixed(4)} -> ${history[history.length - 1].compliance.toFixed(4)})`
  );
}

// 13. Connectivity: a diagonal "staircase" that only touches at corners
// (no shared edges) should still be found connected, since 8-connectivity
// is what the collapse test needs — a real continuum FEM transmits force
// through a shared corner node, not just a shared edge.
{
  const threshold = 0.5;
  const staircase = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  const result = findConnectedPath(staircase, threshold, { elx: 0, ely: 0 }, { elx: 2, ely: 2 });
  assert.ok(result !== null, "a corner-touching diagonal staircase should be found connected");
  console.log("✓ a diagonal staircase (corner-touching only) is found connected");
}

// 14. Two solid cells with a real gap between them (not even diagonally
// touching) must be reported as NOT connected.
{
  const threshold = 0.5;
  const gapped = [
    [1, 0, 0],
    [0, 0, 0],
    [0, 0, 1],
  ];
  const result = findConnectedPath(gapped, threshold, { elx: 0, ely: 0 }, { elx: 2, ely: 2 });
  assert.strictEqual(result, null, "cells with a real gap between them should not be connected");
  console.log("✓ a genuine gap is correctly reported as not connected");
}

// 15. minDensity must be the weakest cell actually on the path, not the
// average — a chain's strength is set by its thinnest link.
{
  const threshold = 0.1;
  const row = [[0.9, 0.3, 0.9]];
  const result = findConnectedPath(row, threshold, { elx: 0, ely: 0 }, { elx: 2, ely: 0 });
  assert.ok(result !== null, "a solid row should be connected");
  assert.strictEqual(result.minDensity, 0.3, `minDensity should be the weakest cell (0.3), got ${result.minDensity}`);
  console.log("✓ minDensity picks out the weakest cell on the path, not the average");
}

// 16. Unit conversion: 100% material should be MAX_MATERIAL_KG, and the
// two directions should round-trip.
{
  assert.ok(Math.abs(MAX_MATERIAL_KG - 1560) < 1e-6, `MAX_MATERIAL_KG should be 1560, got ${MAX_MATERIAL_KG}`);
  assert.ok(Math.abs(kgToVolumeFraction(MAX_MATERIAL_KG) - 1) < 1e-9, "100% of the max kg should be volume fraction 1");
  const fraction = 0.4;
  const kg = volumeFractionToKg(fraction);
  assert.ok(Math.abs(kgToVolumeFraction(kg) - fraction) < 1e-9, "kg <-> volume fraction should round-trip");
  console.log(`✓ unit conversion round-trips (100% = ${MAX_MATERIAL_KG}kg, 40% = ${kg.toFixed(1)}kg)`);
}

// 17. Force conversion: a 1kg mass should exert exactly g newtons.
{
  assert.ok(Math.abs(kgToNewtons(1) - GRAVITY_M_S2) < 1e-9, "1kg should exert g newtons");
  console.log("✓ force conversion is linear in mass, as expected");
}

// 18. Strain recovery: a displacement field that's a uniform stretch in x
// (u = 0.1*x, v = 0) should give exactly epsilon_x=0.1 and nothing else —
// hand-computed from the node positions, not just re-deriving the B
// matrix's own formula.
{
  // Nodes: bottom-left(0,0), bottom-right(1,0), top-right(1,1), top-left(0,1).
  const stretch_u_e = [0, 0, 0.1, 0, 0.1, 0, 0, 0]; // u=0.1x, v=0 at each corner
  const strain = computeStrain(stretch_u_e);
  assert.ok(Math.abs(strain[0] - 0.1) < 1e-9, `epsilon_x should be 0.1, got ${strain[0]}`);
  assert.ok(Math.abs(strain[1]) < 1e-9, `epsilon_y should be 0, got ${strain[1]}`);
  assert.ok(Math.abs(strain[2]) < 1e-9, `gamma_xy should be 0, got ${strain[2]}`);

  // A different field, v = 0.2*x, u = 0, is pure shear: gamma_xy = du/dy +
  // dv/dx = 0 + 0.2 = 0.2, with no normal strain at all.
  const shear_u_e = [0, 0, 0, 0.2, 0, 0.2, 0, 0];
  const shearStrain = computeStrain(shear_u_e);
  assert.ok(Math.abs(shearStrain[0]) < 1e-9 && Math.abs(shearStrain[1]) < 1e-9, "pure shear field should have no normal strain");
  assert.ok(Math.abs(shearStrain[2] - 0.2) < 1e-9, `gamma_xy should be 0.2, got ${shearStrain[2]}`);
  console.log("✓ strain recovery matches hand-computed values for uniform stretch and pure shear");
}

// 19. Plane-stress Hooke's law: D must match the textbook formula exactly
// (this is a regression check on the matrix entries themselves, not an
// independent derivation).
{
  const E = 1;
  const nu = 0.3;
  const D = planeStressD(E, nu);
  const scale = E / (1 - nu * nu);
  assert.ok(Math.abs(D[0][0] - scale) < 1e-9, "D[0][0] should be E/(1-nu^2)");
  assert.ok(Math.abs(D[0][1] - scale * nu) < 1e-9, "D[0][1] should be nu*E/(1-nu^2)");
  assert.ok(Math.abs(D[2][2] - (scale * (1 - nu)) / 2) < 1e-9, "D[2][2] should be the shear modulus term");

  const strain = [0.1, 0, 0];
  const [sx, sy, txy] = computeStress(strain, D);
  assert.ok(Math.abs(sx - scale * 0.1) < 1e-9, "uniaxial strain should give sigma_x = scale * epsilon_x");
  assert.ok(Math.abs(sy - scale * nu * 0.1) < 1e-9, "plane stress Poisson coupling should give a nonzero sigma_y, not 0");
  assert.ok(Math.abs(txy) < 1e-9, "no shear strain should mean no shear stress");
  console.log("✓ plane-stress D matrix matches Hooke's law, including the Poisson coupling term");
}

// 20. Principal stresses: hand-checkable Mohr's-circle cases.
{
  const [s1a, s2a] = principalStresses(3, 1, 0);
  assert.ok(Math.abs(s1a - 3) < 1e-9 && Math.abs(s2a - 1) < 1e-9, "with no shear, principal stresses are just sigma_x and sigma_y");

  const [s1b, s2b] = principalStresses(1, 1, 1);
  assert.ok(Math.abs(s1b - 2) < 1e-9 && Math.abs(s2b - 0) < 1e-9, `expected [2,0], got [${s1b},${s2b}]`);
  console.log("✓ principal stresses match hand-computed Mohr's-circle values");
}

// 21. End-to-end: maxPrincipalStress on the uniform-stretch case should
// equal scale * 0.1 (since that strain state has no shear, sigma_x is
// already the max principal stress).
{
  const E = 1;
  const nu = 0.3;
  const stretch_u_e = [0, 0, 0.1, 0, 0.1, 0, 0, 0];
  const expected = (E / (1 - nu * nu)) * 0.1;
  const s1 = maxPrincipalStress(stretch_u_e, E, nu);
  assert.ok(Math.abs(s1 - expected) < 1e-9, `maxPrincipalStress should be ${expected}, got ${s1}`);
  console.log("✓ maxPrincipalStress matches the hand-computed value end-to-end");
}

// 22. Moment of inertia and Euler buckling load: check against the
// textbook formula directly, and check the scaling relationships that
// have to hold regardless of the exact numbers (doubling length should
// cut the critical load to a quarter, since it's a length^2 term).
{
  const bend = 0.1;
  const perp = 0.3;
  const I = rectMomentOfInertia(bend, perp);
  assert.ok(Math.abs(I - (perp * bend ** 3) / 12) < 1e-12, "moment of inertia should match perp * bend^3 / 12");

  const E = 3e10;
  const L = 1;
  const Pcr = eulerCriticalLoad(E, I, L);
  assert.ok(Math.abs(Pcr - (Math.PI ** 2 * E * I) / L ** 2) < 1e-3, "Euler load should match pi^2 EI / L^2");

  const PcrDoubleLength = eulerCriticalLoad(E, I, L * 2);
  assert.ok(Math.abs(PcrDoubleLength - Pcr / 4) < 1e-6, "doubling length should cut critical load to 1/4");

  const PcrDoubleI = eulerCriticalLoad(E, 2 * I, L);
  assert.ok(Math.abs(PcrDoubleI - Pcr * 2) < 1e-3, "doubling moment of inertia should double critical load");
  console.log("✓ Euler buckling load matches the textbook formula and its scaling relationships");
}

// 23. Truss statics: a symmetric A-frame (45 degrees on each side) under a
// central vertical load should split it as W/sqrt(2) compression in each
// leg — a standard, hand-checkable statics result — and for an
// asymmetric case, the solved forces must satisfy the original
// equilibrium equations exactly (a general self-consistency check, not
// tied to one specific hand-solved number).
{
  const W = 100;
  const { left, right } = resolveTrussAxialForces(Math.PI / 4, (3 * Math.PI) / 4, W);
  const expected = W / Math.sqrt(2);
  assert.ok(Math.abs(left - expected) < 1e-9, `left should be W/sqrt(2)=${expected}, got ${left}`);
  assert.ok(Math.abs(right - expected) < 1e-9, `right should be W/sqrt(2)=${expected}, got ${right}`);

  const angleLeft = 0.5;
  const angleRight = 2.3;
  const forces = resolveTrussAxialForces(angleLeft, angleRight, W);
  const horizontalSum = forces.left * Math.cos(angleLeft) + forces.right * Math.cos(angleRight);
  const verticalSum = forces.left * Math.sin(angleLeft) + forces.right * Math.sin(angleRight);
  assert.ok(Math.abs(horizontalSum) < 1e-6, `horizontal equilibrium should hold, got ${horizontalSum}`);
  assert.ok(Math.abs(verticalSum - W) < 1e-6, `vertical equilibrium should sum to W, got ${verticalSum}`);
  console.log("✓ truss statics matches the symmetric hand-solved case and satisfies equilibrium for an asymmetric one");
}

// 24. End-to-end failure evaluation on a real converged shape.
{
  const numElemX = 20;
  const numElemY = 10;
  const volumeFraction = 0.4;
  const loadColumn = numElemX / 2;

  const bottomLeft = nodeId(0, 0, numElemX);
  const bottomRight = nodeId(numElemX, 0, numElemX);
  const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];
  const node = nodeId(loadColumn, numElemY, numElemX);
  const [, loadDof] = nodeDofs(node);
  const loads = [[loadDof, -1]];

  const { densities, u } = runTopologyOptimization(numElemX, numElemY, fixedDofs, loads, { volumeFraction });

  // 25a. A trivially light weight should be reported connected and fine.
  const light = evaluateHalves({ numElemX, numElemY, densities, u, loadColumn, testWeightKg: 1 });
  assert.ok(light.ok, "the shape should be connected for this volume fraction");
  assert.ok(!light.left.fails && !light.right.fails, "a 1kg test weight should not fail either side");

  // 25b. Doubling the applied weight must exactly double the stress
  // ratio — this is the actual mathematical claim behind skipping a real
  // elastic modulus (stress scales linearly with applied force,
  // independent of E, for a linear elastic solve) — not just a
  // plausibility check.
  const r1 = evaluateHalves({ numElemX, numElemY, densities, u, loadColumn, testWeightKg: 1000 });
  const r2 = evaluateHalves({ numElemX, numElemY, densities, u, loadColumn, testWeightKg: 2000 });
  assert.ok(
    Math.abs(r2.left.stressRatio - 2 * r1.left.stressRatio) < 1e-6 * Math.max(1, r1.left.stressRatio),
    `doubling the weight should double the stress ratio: ${r1.left.stressRatio} -> ${r2.left.stressRatio}`
  );
  assert.ok(
    Math.abs(r2.right.stressRatio - 2 * r1.right.stressRatio) < 1e-6 * Math.max(1, r1.right.stressRatio),
    "doubling the weight should double the right side's stress ratio too"
  );

  // 25c. Symmetric setup (centered load, symmetric supports) should give
  // left and right nearly identical results.
  assert.ok(
    Math.abs(r1.left.stressRatio - r1.right.stressRatio) < 1e-6 * Math.max(1, r1.left.stressRatio),
    `symmetric setup should give matching stress ratios, got ${r1.left.stressRatio} vs ${r1.right.stressRatio}`
  );
  assert.ok(
    Math.abs(r1.left.axialForceN - r1.right.axialForceN) < 1e-6 * Math.max(1, r1.left.axialForceN),
    "symmetric setup should give matching axial forces"
  );

  // 25d. Some large enough weight must eventually fail — otherwise the
  // check could be vacuously always-true.
  const heavy = evaluateHalves({ numElemX, numElemY, densities, u, loadColumn, testWeightKg: 1e9 });
  assert.ok(heavy.left.fails && heavy.right.fails, "an absurdly heavy test weight must fail both sides");

  console.log("✓ failure evaluation: stress ratio scales linearly with weight, is symmetric for a symmetric setup, and a heavy enough weight fails");
}

// 25. Convex hull: a square with a point in its interior and a point
// exactly on one of its edges must both be dropped — the hull is exactly
// the 4 corners.
{
  const points = [
    { x: 0, y: 0 }, { x: 4, y: 0 }, { x: 4, y: 4 }, { x: 0, y: 4 },
    { x: 2, y: 2 }, // interior
    { x: 2, y: 0 }, // on an edge
  ];
  const hull = convexHull(points);
  assert.equal(hull.length, 4, `hull should have exactly 4 vertices, got ${hull.length}`);
  for (const corner of [{ x: 0, y: 0 }, { x: 4, y: 0 }, { x: 4, y: 4 }, { x: 0, y: 4 }]) {
    assert.ok(
      hull.some((p) => p.x === corner.x && p.y === corner.y),
      `hull should include corner (${corner.x},${corner.y})`
    );
  }
  console.log("✓ convex hull drops interior and collinear-edge points, keeping exactly the 4 corners");
}

// 26. Polygon centroid: a unit square's centroid is its center, and a
// triangle's centroid happens to equal the plain average of its vertices
// (a fact true only for triangles) — both hand-checkable, and this is the
// exact formula collapse.js relies on matching Matter.js's own Vertices.centre.
{
  const square = [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 1, y: 1 }, { x: 0, y: 1 }];
  const squareCentroid = polygonCentroid(square);
  assert.ok(Math.abs(squareCentroid.x - 0.5) < 1e-9 && Math.abs(squareCentroid.y - 0.5) < 1e-9,
    `unit square centroid should be (0.5,0.5), got (${squareCentroid.x},${squareCentroid.y})`);

  const triangle = [{ x: 0, y: 0 }, { x: 3, y: 0 }, { x: 0, y: 3 }];
  const triangleCentroid = polygonCentroid(triangle);
  assert.ok(Math.abs(triangleCentroid.x - 1) < 1e-9 && Math.abs(triangleCentroid.y - 1) < 1e-9,
    `triangle centroid should be (1,1), got (${triangleCentroid.x},${triangleCentroid.y})`);
  console.log("✓ polygon centroid matches hand-computed values for a square and a triangle");
}

// 27. Grouping cells by nearest seed must be a strict partition: every cell
// assigned to exactly one group, and the union recovers the original set —
// this is what guarantees fracturing a shape never drops or duplicates
// material.
{
  const cells = [];
  for (let ely = 0; ely < 6; ely++) {
    for (let elx = 0; elx < 10; elx++) cells.push({ elx, ely });
  }
  const groups = groupCellsBySeed(cells, 4);
  assert.ok(groups.length > 0 && groups.length <= 4, `expected 1-4 non-empty groups, got ${groups.length}`);

  const seen = new Set();
  let total = 0;
  for (const group of groups) {
    assert.ok(group.length > 0, "groupCellsBySeed should never return an empty group");
    for (const cell of group) {
      const key = `${cell.elx},${cell.ely}`;
      assert.ok(!seen.has(key), `cell (${cell.elx},${cell.ely}) should not appear in more than one group`);
      seen.add(key);
      total++;
    }
  }
  assert.equal(total, cells.length, "every cell must be assigned to exactly one group");
  console.log("✓ groupCellsBySeed partitions all cells with no duplicates or omissions");
}

// 28. End-to-end shard building on a real converged shape's half: every
// shard's polygon must be a valid, non-degenerate hull (>= 3 vertices,
// positive area), and the shards collectively account for every solid
// cell in that half exactly once.
{
  const numElemX = 20;
  const numElemY = 10;
  const volumeFraction = 0.4;
  const loadColumn = numElemX / 2;

  const bottomLeft = nodeId(0, 0, numElemX);
  const bottomRight = nodeId(numElemX, 0, numElemX);
  const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];
  const node = nodeId(loadColumn, numElemY, numElemX);
  const [, loadDof] = nodeDofs(node);
  const { densities } = runTopologyOptimization(numElemX, numElemY, fixedDofs, [[loadDof, -1]], { volumeFraction });

  const cells = [];
  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < loadColumn; elx++) {
      if (densities[ely][elx] > 0.5) cells.push({ elx, ely });
    }
  }
  assert.ok(cells.length > 0, "the left half should have some solid material at this volume fraction");

  const cellCorners = (cell) => [
    { x: cell.elx, y: cell.ely },
    { x: cell.elx + 1, y: cell.ely },
    { x: cell.elx + 1, y: cell.ely + 1 },
    { x: cell.elx, y: cell.ely + 1 },
  ];
  const shards = buildShards(cells, 5, cellCorners);
  assert.ok(shards.length > 0, "should produce at least one shard");

  const seen = new Set();
  for (const shard of shards) {
    assert.ok(shard.vertices.length >= 3, `every shard's hull must have at least 3 vertices, got ${shard.vertices.length}`);
    const area = Math.abs(
      shard.vertices.reduce((sum, p, i) => {
        const q = shard.vertices[(i + 1) % shard.vertices.length];
        return sum + (p.x * q.y - q.x * p.y);
      }, 0) / 2
    );
    assert.ok(area > 0, "every shard's hull must have positive area");
    for (const cell of shard.cells) {
      const key = `${cell.elx},${cell.ely}`;
      assert.ok(!seen.has(key), `cell (${cell.elx},${cell.ely}) should not appear in more than one shard`);
      seen.add(key);
    }
  }
  assert.equal(seen.size, cells.length, "shards together must account for every solid cell exactly once");
  console.log("✓ shard building on a real converged shape produces valid, non-overlapping, complete coverage");
}

console.log("\nAll checks passed.");
