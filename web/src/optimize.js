import { solveDisplacement } from "./structure.js";
import { computeSensitivities } from "./sensitivity.js";
import { filterSensitivities } from "./filter.js";
import { updateDensities } from "./oc.js";

// Compliance = f . u, the load dotted with the resulting displacement — a
// measure of how much the structure "gives" under the load (lower is
// stiffer, better). This is the actual quantity the optimizer is minimizing,
// so whether it decreases and settles is the real test of correctness — a
// far more reliable signal than whether the resulting shape *looks* like a
// clean truss, which depends on mesh resolution and volume fraction and can
// be misleading.
function computeCompliance(loads, u) {
  return loads.reduce((sum, [dof, value]) => sum + value * u[dof], 0);
}

// The full topology-optimization loop, one iteration per yield: solve for
// displacement under the current material distribution, work out which
// elements matter (sensitivity), smooth that out (filter), and shift
// material toward the elements that matter most (OC update). Exposed as a
// generator — rather than a function that just returns the final result —
// so a renderer can draw every intermediate step (watching the structure
// take shape) instead of only ever seeing the converged end state.
export function* iterateTopologyOptimization(numElemX, numElemY, fixedDofs, loads, options = {}) {
  const {
    volumeFraction = 0.5,
    rmin = 1.5,
    move = 0.2,
    maxIterations = 100,
    tolerance = 0.01,
  } = options;

  let densities = Array.from({ length: numElemY }, () => new Array(numElemX).fill(volumeFraction));
  let previousU;

  for (let iteration = 0; iteration < maxIterations; iteration++) {
    const u = solveDisplacement(numElemX, numElemY, densities, fixedDofs, loads, { previousU });
    previousU = u;
    const compliance = computeCompliance(loads, u);
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
    const converged = maxChange < tolerance;
    yield { densities, iteration: iteration + 1, compliance, maxChange, converged };

    if (converged) return;
  }
}

// Drains iterateTopologyOptimization synchronously and returns just the
// final result — what tests and the Node demo script want, when nothing is
// watching the intermediate frames.
export function runTopologyOptimization(numElemX, numElemY, fixedDofs, loads, options = {}) {
  const history = [];
  let last;

  for (const step of iterateTopologyOptimization(numElemX, numElemY, fixedDofs, loads, options)) {
    history.push({ iteration: step.iteration, compliance: step.compliance, maxChange: step.maxChange });
    last = step;
  }

  return {
    densities: last.densities,
    iterations: last.iteration,
    converged: last.converged,
    history,
  };
}
