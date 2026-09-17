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

console.log("\nAll checks passed.");
