import { solveDisplacement } from "./structure.js";
import { computeSensitivities } from "./sensitivity.js";
import { filterSensitivities } from "./filter.js";
import { updateDensities } from "./oc.js";

// The full topology-optimization loop: solve for displacement under the
// current material distribution, work out which elements matter
// (sensitivity), smooth that out (filter), and shift material toward the
// elements that matter most (OC update) — repeating until the design stops
// changing much, or a safety cap on iterations is hit.
export function runTopologyOptimization(numElemX, numElemY, fixedDofs, loads, options = {}) {
  const {
    volumeFraction = 0.5,
    rmin = 1.5,
    move = 0.2,
    maxIterations = 100,
    tolerance = 0.01,
  } = options;

  let densities = Array.from({ length: numElemY }, () => new Array(numElemX).fill(volumeFraction));

  for (let iteration = 0; iteration < maxIterations; iteration++) {
    const u = solveDisplacement(numElemX, numElemY, densities, fixedDofs, loads);
    const dc = computeSensitivities(numElemX, numElemY, densities, u);
    const dcFiltered = filterSensitivities(numElemX, numElemY, densities, dc, rmin);
    const newDensities = updateDensities(numElemX, numElemY, densities, dcFiltered, volumeFraction, move);

    let maxChange = 0;
    for (let ely = 0; ely < numElemY; ely++) {
      for (let elx = 0; elx < numElemX; elx++) {
        maxChange = Math.max(maxChange, Math.abs(newDensities[ely][elx] - densities[ely][elx]));
      }
    }

    densities = newDensities;

    if (maxChange < tolerance) {
      return { densities, iterations: iteration + 1, converged: true };
    }
  }

  return { densities, iterations: maxIterations, converged: false };
}
