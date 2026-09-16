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
import { getElementStiffnessMatrix, matVec8 } from "./web/src/fem.js";

const EPSILON = 1e-9;

function assertApproxZero(vector, label) {
  vector.forEach((value, i) => {
    assert.ok(
      Math.abs(value) < EPSILON,
      `${label}[${i}] should be ~0, got ${value}`
    );
  });
}

const KE = getElementStiffnessMatrix();

// 1. Symmetry.
for (let i = 0; i < 8; i++) {
  for (let j = 0; j < 8; j++) {
    assert.ok(
      Math.abs(KE[i][j] - KE[j][i]) < EPSILON,
      `KE[${i}][${j}]=${KE[i][j]} should equal KE[${j}][${i}]=${KE[j][i]}`
    );
  }
}
console.log("✓ KE is symmetric");

// 2. Rigid-body translation in x: every node moves +1 in x, 0 in y.
const translateX = [1, 0, 1, 0, 1, 0, 1, 0];
assertApproxZero(matVec8(KE, translateX), "KE * translateX");
console.log("✓ rigid-body translation in x produces ~zero force");

// 3. Rigid-body translation in y: every node moves +1 in y, 0 in x.
const translateY = [0, 1, 0, 1, 0, 1, 0, 1];
assertApproxZero(matVec8(KE, translateY), "KE * translateY");
console.log("✓ rigid-body translation in y produces ~zero force");

console.log("\nAll checks passed.");
