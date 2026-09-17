import { assembleGlobalStiffness } from "./assemble.js";
import { numDofs } from "./mesh.js";
import { reduceSystem, expandSolution } from "./boundary.js";
import { solveLinearSystem } from "./linalg.js";

// Solves for the displacement of every node under the given loads.
//
//   fixedDofs — DOF indices whose displacement is pinned to 0 (supports).
//   loads     — array of [dof, value] pairs; every other DOF gets 0 force.
//
// Returns the full displacement vector (length numDofs(numElemX, numElemY)),
// indexed the same way as the DOFs everywhere else in this project.
export function solveDisplacement(numElemX, numElemY, densities, fixedDofs, loads) {
  const K = assembleGlobalStiffness(numElemX, numElemY, densities);
  const n = numDofs(numElemX, numElemY);

  const f = new Array(n).fill(0);
  for (const [dof, value] of loads) {
    f[dof] = value;
  }

  const { Kff, ff, freeDofs } = reduceSystem(K, f, fixedDofs);
  const uFree = solveLinearSystem(Kff, ff);
  return expandSolution(uFree, freeDofs, n);
}
