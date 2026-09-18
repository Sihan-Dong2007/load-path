// Calibration sweep, redone for the stress+buckling failure model
// (failure.js) — the old area-approximation numbers are no longer
// meaningful now that the physics changed completely. For a few load
// positions, checks whether candidate slider ranges (material kg, test
// weight kg) actually reach all three regimes: always fails,
// position-dependent, always holds.
import { nodeId, nodeDofs } from "./web/src/mesh.js";
import { runTopologyOptimization } from "./web/src/optimize.js";
import { evaluateHalves } from "./web/src/failure.js";
import { kgToVolumeFraction, MAX_MATERIAL_KG } from "./web/src/units.js";

const numElemX = 60;
const numElemY = 30;

const bottomLeft = nodeId(0, 0, numElemX);
const bottomRight = nodeId(numElemX, 0, numElemX);
const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

// Shape (and its displacement field) only depends on (loadColumn,
// materialKg) — compute once per pair, then cheaply check every weightKg
// against it.
function computeShape(loadColumn, materialKg) {
  const volumeFraction = kgToVolumeFraction(materialKg);
  const node = nodeId(loadColumn, numElemY, numElemX);
  const [, loadDof] = nodeDofs(node);
  const { densities, u } = runTopologyOptimization(numElemX, numElemY, fixedDofs, [[loadDof, -1]], { volumeFraction });
  return { densities, u };
}

function outcome(shape, loadColumn, weightKg) {
  const result = evaluateHalves({ numElemX, numElemY, densities: shape.densities, u: shape.u, loadColumn, testWeightKg: weightKg });
  if (!result.ok) return "disc. ";
  if (!result.left.fails && !result.right.fails) return "holds ";
  const reason = (h) => (h.bucklingFails ? "buckle" : h.strengthFails ? "stress" : "-");
  return `BREAK(L:${reason(result.left)}/R:${reason(result.right)})`;
}

console.log(`MAX_MATERIAL_KG = ${MAX_MATERIAL_KG}`);

const positions = {
  center: Math.round(numElemX / 2),
  offCenter: Math.round(numElemX * 0.3),
  nearSupport: Math.round(numElemX * 0.1),
};

const materials = [250, 400, 620, 900, 1200];
const weights = [2000, 5000, 10000, 30000, 100000, 300000, 1000000, 3000000, 10000000];

for (const materialKg of materials) {
  const shapes = {};
  for (const [name, col] of Object.entries(positions)) shapes[name] = computeShape(col, materialKg);

  for (const weightKg of weights) {
    const results = Object.entries(positions).map(([name, col]) => `${name}=${outcome(shapes[name], col, weightKg)}`);
    console.log(`material=${materialKg}kg weight=${weightKg}kg -> ${results.join(", ")}`);
  }
}
