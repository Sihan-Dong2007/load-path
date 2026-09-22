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
import { computeSensitivities, computeElementWork } from "./web/src/sensitivity.js";
import { filterSensitivities } from "./web/src/filter.js";
import { updateDensities } from "./web/src/oc.js";
import { assembleSparseStiffness, sparseMatVec } from "./web/src/sparse.js";
import { conjugateGradient } from "./web/src/cg.js";
import { reduceSystem } from "./web/src/boundary.js";
import { solveLinearSystem } from "./web/src/linalg.js";
import { runTopologyOptimization, iterateTopologyOptimization } from "./web/src/optimize.js";
import { findConnectedPath } from "./web/src/connectivity.js";
import { computeStrain, planeStressD, computeStress, principalStresses, maxPrincipalStress } from "./web/src/stress.js";
import { rectMomentOfInertia, eulerCriticalLoad, resolveTrussAxialForces } from "./web/src/buckling.js";
import { evaluateHalves, loadCapacityKg } from "./web/src/failure.js";
import { nodePosition, deformScaleFor } from "./web/src/deform.js";
import { DOMAIN } from "./web/src/scene.js";
import { createDesign, countCells, budgetCells, paintBrush, toDensities, supportsConnected, connectionStatus, connectionHint, VOID_DENSITY } from "./web/src/design.js";
import { convexHull, polygonCentroid, groupCellsBySeed, buildShards } from "./web/src/fracture.js";
import { kgToVolumeFraction, volumeFractionToKg, MAX_MATERIAL_KG, kgToNewtons, GRAVITY_M_S2, stressScaleFactor, BRIDGE_DEPTH_M } from "./web/src/units.js";
import { elementDofs } from "./web/src/mesh.js";

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

// 29. Real-unit stress scaling, checked against a textbook case: a uniform
// bar under total axial force F has stress F / (thickness * height). The
// FEM runs in unit-model units, so model stress x F x stressScaleFactor(h)
// must reproduce that in pascals. This is the test that would have caught
// the missing 1/(t*h) factor (~100x for the real bridge) the strength
// check originally had — none of the other tests touched absolute stress.
{
  const nx = 8;
  const ny = 4;
  const cellSizeM = 0.05; // real element size
  const forceN = 1000;

  const densities = solidDensities(nx, ny);
  const fixed = [];
  for (let r = 0; r <= ny; r++) fixed.push(...nodeDofs(nodeId(0, r, nx)));
  const loads = [];
  for (let r = 0; r <= ny; r++) {
    const share = r === 0 || r === ny ? 0.5 : 1; // consistent nodal loads on the end edge
    loads.push([nodeDofs(nodeId(nx, r, nx))[0], share / ny]);
  }
  const u = solveDisplacement(nx, ny, densities, fixed, loads);
  const u_e = elementDofs(nx / 2, ny / 2, nx).map((d) => u[d]);

  const realStressPa = maxPrincipalStress(u_e, 1, 0.3) * forceN * stressScaleFactor(cellSizeM);
  const textbookPa = forceN / (BRIDGE_DEPTH_M * ny * cellSizeM);
  assert.ok(
    Math.abs(realStressPa - textbookPa) / textbookPa < 0.01,
    `real stress should match F/(t*H)=${textbookPa.toFixed(1)} Pa within 1%, got ${realStressPa.toFixed(1)} Pa`
  );
  console.log(`✓ real-unit stress matches the textbook F/(t·H) for a uniform bar (${realStressPa.toFixed(0)} Pa vs ${textbookPa.toFixed(0)} Pa)`);
}

// 30. The per-element work the growth heat map shows must sum to exactly the
// compliance (f . u) the optimizer minimizes — on a non-uniform design, so a
// density^penal mistake couldn't hide behind uniform densities.
{
  const numElemX = 8;
  const numElemY = 4;
  const densities = Array.from({ length: numElemY }, (_, ely) =>
    Array.from({ length: numElemX }, (_, elx) => 0.3 + (0.7 * ((elx * 7 + ely * 3) % 5)) / 4)
  );
  const fixedDofs = [...nodeDofs(nodeId(0, 0, numElemX)), ...nodeDofs(nodeId(numElemX, 0, numElemX))];
  const [, loadDof] = nodeDofs(nodeId(numElemX / 2, numElemY, numElemX));
  const u = solveDisplacement(numElemX, numElemY, densities, fixedDofs, [[loadDof, -1]]);

  const totalWork = computeElementWork(numElemX, numElemY, densities, u)
    .flat()
    .reduce((sum, w) => sum + w, 0);
  const compliance = -1 * u[loadDof]; // f . u for a single unit downward load
  assert.ok(
    Math.abs(totalWork - compliance) < 1e-6 * Math.abs(compliance),
    `element work should sum to compliance ${compliance}, got ${totalWork}`
  );
  console.log(`✓ per-element strain energy sums to exactly the compliance (${totalWork.toFixed(6)})`);
}

// 31. Hitting the iteration cap without converging must still hand the caller
// one final frame (finished, with a displacement field matching the densities
// it returned) — otherwise the page waits forever for a "converged" frame that
// never comes. maxIterations=2 with a tight tolerance can't converge.
{
  const numElemX = 20;
  const numElemY = 10;
  const fixedDofs = [...nodeDofs(nodeId(0, 0, numElemX)), ...nodeDofs(nodeId(numElemX, 0, numElemX))];
  const [, loadDof] = nodeDofs(nodeId(numElemX / 2, numElemY, numElemX));

  const steps = [...iterateTopologyOptimization(numElemX, numElemY, fixedDofs, [[loadDof, -1]], {
    volumeFraction: 0.4,
    maxIterations: 2,
    tolerance: 1e-12,
  })];
  const last = steps[steps.length - 1];
  assert.equal(steps.length, 2, "should stop at maxIterations");
  assert.ok(last.finished && !last.converged, "the capped last frame is finished but not converged");
  assert.ok(steps.slice(0, -1).every((s) => !s.finished), "earlier frames are not finished");

  const resolved = solveDisplacement(numElemX, numElemY, last.densities, fixedDofs, [[loadDof, -1]]);
  assert.ok(
    resolved.every((v, i) => Math.abs(v - last.u[i]) < 1e-6 * (1 + Math.abs(v))),
    "the final frame's u must match the densities it carries"
  );
  console.log("✓ hitting the iteration cap still yields a final, consistent frame");
}

// 32. Drawing the deformed mesh: node positions must follow the displacement,
// with the FEM's y-up flipped to the canvas's y-down. A synthetic
// displacement pins the exact arithmetic; a real solve then confirms a
// downward load really draws the loaded node lower on screen.
{
  const nx = 4;
  const ny = 2;
  const zero = new Array(numDofs(nx, ny)).fill(0);
  const rest = nodePosition(0, 0, nx, ny, zero, 5);
  assert.ok(rest.x === DOMAIN.x && rest.y === DOMAIN.y + DOMAIN.h, "at rest, node (0,0) sits at the domain's bottom-left");

  const synthetic = new Array(numDofs(nx, ny)).fill(0);
  const [dofX, dofY] = nodeDofs(nodeId(2, 1, nx));
  synthetic[dofX] = 1;
  synthetic[dofY] = -2; // 2 units DOWN in FEM terms
  const base = nodePosition(2, 1, nx, ny, zero, 10);
  const moved = nodePosition(2, 1, nx, ny, synthetic, 10);
  assert.ok(Math.abs(moved.x - base.x - 10) < 1e-9, "ux=+1 at scale 10 moves the node 10px right");
  assert.ok(Math.abs(moved.y - base.y - 20) < 1e-9, "uy=-2 (down) at scale 10 moves the node 20px DOWN the screen (larger y)");

  const scale = deformScaleFor(synthetic, 0.1);
  assert.ok(Math.abs(scale * 2 - 0.1 * DOMAIN.h) < 1e-9, "the largest displacement (2) must draw as exactly 10% of the domain height");

  const fixed = [...nodeDofs(nodeId(0, 0, nx)), ...nodeDofs(nodeId(nx, 0, nx))];
  const [, loadDof] = nodeDofs(nodeId(nx / 2, ny, nx));
  const u = solveDisplacement(nx, ny, solidDensities(nx, ny), fixed, [[loadDof, -1]]);
  const s = deformScaleFor(u, 0.1);
  const loadedRest = nodePosition(nx / 2, ny, nx, ny, u, 0);
  const loadedBent = nodePosition(nx / 2, ny, nx, ny, u, s);
  assert.ok(loadedBent.y > loadedRest.y, "a downward load must draw the loaded node lower on screen");
  console.log("✓ deformed-mesh geometry follows the displacement, y-flip included");
}

// 33. Load capacity is exact: just under it every half holds, just over it
// something fails, and it scales with nothing the test weight touches. Run on
// a real optimized shape so the strength and buckling paths are both live.
{
  const numElemX = 20;
  const numElemY = 10;
  const loadColumn = numElemX / 2;
  const fixedDofs = [...nodeDofs(nodeId(0, 0, numElemX)), ...nodeDofs(nodeId(numElemX, 0, numElemX))];
  const [, loadDof] = nodeDofs(nodeId(loadColumn, numElemY, numElemX));
  const { densities, u } = runTopologyOptimization(numElemX, numElemY, fixedDofs, [[loadDof, -1]], { volumeFraction: 0.4 });

  const capacity = loadCapacityKg({ numElemX, numElemY, densities, u, loadColumn });
  assert.ok(Number.isFinite(capacity) && capacity > 0, `capacity should be a positive number, got ${capacity}`);

  const under = evaluateHalves({ numElemX, numElemY, densities, u, loadColumn, testWeightKg: capacity * 0.999 });
  const over = evaluateHalves({ numElemX, numElemY, densities, u, loadColumn, testWeightKg: capacity * 1.001 });
  assert.ok(!under.left.fails && !under.right.fails, "just under the capacity, both halves must hold");
  assert.ok(over.left.fails || over.right.fails, "just over the capacity, a half must fail");

  const disconnected = solidDensities(numElemX, numElemY).map((row) => row.map(() => 0));
  assert.equal(loadCapacityKg({ numElemX, numElemY, densities: disconnected, u, loadColumn }), 0, "no material, no capacity");
  console.log(`✓ load capacity is exact: holds at 99.9% of ${capacity.toFixed(0)} kg, fails at 100.1%`);
}

// 34. The optimizer's per-iteration replay state must be self-consistent: each
// iteration's densities are the next one's starting (solved) densities, and
// the solved displacement really is the solution for the solved densities.
{
  const numElemX = 12;
  const numElemY = 6;
  const fixedDofs = [...nodeDofs(nodeId(0, 0, numElemX)), ...nodeDofs(nodeId(numElemX, 0, numElemX))];
  const [, loadDof] = nodeDofs(nodeId(numElemX / 2, numElemY, numElemX));
  const steps = [...iterateTopologyOptimization(numElemX, numElemY, fixedDofs, [[loadDof, -1]], { volumeFraction: 0.4, maxIterations: 4, tolerance: 1e-12 })];
  assert.equal(steps.length, 4);
  for (let i = 1; i < steps.length; i++) {
    assert.deepEqual(steps[i].solvedDensities, steps[i - 1].densities, `iteration ${i + 1} must start from iteration ${i}'s result`);
  }
  const check = solveDisplacement(numElemX, numElemY, steps[2].solvedDensities, fixedDofs, [[loadDof, -1]]);
  assert.ok(
    check.every((v, i) => Math.abs(v - steps[2].solvedU[i]) < 1e-6 * (1 + Math.abs(v))),
    "solvedU must be the displacement for solvedDensities"
  );
  assert.ok(steps.every((s) => s.importance.length === numElemY && s.importance[0].length === numElemX), "importance is a full grid");
  console.log("✓ each iteration's replay state is self-consistent");
}

// 35. The visitor's design: the stone budget is a hard limit, erasing gives it
// back, and a bridge only counts once BOTH supports reach the weight.
{
  const nx = 12;
  const ny = 6;
  assert.equal(budgetCells(0.4, nx, ny), Math.round(0.4 * nx * ny), "budget = the optimizer's stone, in whole cells");

  const design = createDesign(nx, ny);
  assert.equal(countCells(design), 0);

  // A big brush stroke can't go past the budget.
  paintBrush(design, 6, 3, 5, false, 10);
  assert.equal(countCells(design), 10, "painting stops exactly at the budget");
  assert.equal(paintBrush(design, 6, 3, 5, false, 10), 0, "with the budget spent, more painting changes nothing");

  // Erasing frees stone that can then be spent elsewhere.
  const freed = paintBrush(design, 6, 3, 5, true, 10);
  assert.equal(freed, 10, "the erase stroke removed every painted cell");
  assert.equal(countCells(design), 0);
  assert.equal(paintBrush(design, 0, 0, 0, false, 10), 1, "radius 0 paints exactly one cell");

  // Densities: painted cells solid, everything else the optimizer's "empty".
  const densities = toDensities(design);
  assert.equal(densities[0][0], 1);
  assert.equal(densities[3][3], VOID_DENSITY);

  // Connectivity: an arch from corner to corner reaches the load; a single
  // leg, or two legs that never meet, does not.
  const arch = createDesign(nx, ny);
  const load = 6;
  for (let i = 0; i < ny; i++) {
    arch[i][Math.min(load - 1, i)] = 1; // left leg climbs toward the load
    arch[i][Math.max(load, nx - 1 - i)] = 1; // right leg mirrors it
  }
  assert.ok(supportsConnected(arch, load), "two legs meeting under the weight are connected");
  const oneLeg = arch.map((row, y) => row.map((v, x) => (x >= load ? 0 : v)));
  assert.ok(!supportsConnected(oneLeg, load), "one leg alone is not a bridge");
  assert.ok(!supportsConnected(createDesign(nx, ny), load), "an empty design is not connected");
  console.log("✓ the visitor's design respects the stone budget, and connectivity needs both legs");
}

// 36. The comparison the challenge makes must be apples to apples. Solved by
// the same model, more stone can never sag more (a stiffness sanity check on
// the plumbing), and the optimizer's own shape, thresholded to whole cells,
// is a valid, connected, solvable design.
{
  const nx = 20;
  const ny = 10;
  const load = nx / 2;
  const fixedDofs = [...nodeDofs(nodeId(0, 0, nx)), ...nodeDofs(nodeId(nx, 0, nx))];
  const [, loadDof] = nodeDofs(nodeId(load, ny, nx));
  const sag = (design) => -solveDisplacement(nx, ny, toDensities(design), fixedDofs, [[loadDof, -1]])[loadDof];

  const { densities } = runTopologyOptimization(nx, ny, fixedDofs, [[loadDof, -1]], { volumeFraction: 0.4 });
  const traced = densities.map((row) => row.map((v) => (v > 0.5 ? 1 : 0)));
  assert.ok(supportsConnected(traced, load), "the optimizer's shape, thresholded to whole cells, is connected");

  const tracedSag = sag(traced);
  assert.ok(Number.isFinite(tracedSag) && tracedSag > 0, `a connected design has a finite positive sag, got ${tracedSag}`);

  const fuller = traced.map((row, y) => row.map((v, x) => (v === 1 || (y < 3 && Math.abs(x - load) < 6) ? 1 : 0)));
  assert.ok(countCells(fuller) > countCells(traced));
  assert.ok(sag(fuller) < tracedSag, "adding stone to a design must not make it sag more");
  console.log(`✓ challenge designs are solved by the same model (traced optimizer shape sags ${tracedSag.toFixed(2)}, more stone sags less)`);
}

// 37. The "not connected" hint must name what is actually missing.
{
  const nx = 12;
  const ny = 6;
  const load = 6;
  const empty = createDesign(nx, ny);
  assert.ok(/two marked corner cells/.test(connectionHint(connectionStatus(empty, load))));

  const leftOnly = createDesign(nx, ny);
  leftOnly[0][0] = 1;
  assert.ok(/right corner cell/.test(connectionHint(connectionStatus(leftOnly, load))), "only the left corner is stone: ask for the right one");

  const cornersOnly = createDesign(nx, ny);
  cornersOnly[0][0] = 1;
  cornersOnly[0][nx - 1] = 1;
  assert.ok(/Reach the weight/.test(connectionHint(connectionStatus(cornersOnly, load))), "corners are stone but nothing reaches the weight");

  // A full left leg and a right leg with a gap in it: only the right is named.
  const gapped = createDesign(nx, ny);
  for (let i = 0; i < ny; i++) gapped[i][Math.min(load - 1, i)] = 1;
  gapped[0][nx - 1] = 1;
  gapped[ny - 1][load] = 1; // stone at the top under the weight, but not joined to the right support
  const status = connectionStatus(gapped, load);
  assert.ok(status.left && !status.right, "left joined, right not");
  assert.ok(/right support/.test(connectionHint(status)), "the hint names the right support");
  console.log("\u2713 the not-connected hint names exactly what is missing");
}

console.log("\nAll checks passed.");
