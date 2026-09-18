// Calibration sweep: for a few load positions, checks whether the current
// slider ranges (material kg, test-weight kg) actually reach all three
// regimes — always fails, position-dependent, always holds — using the
// same pass/fail formula the real page uses (units.js), without needing
// the physics engine or a browser at all.
import { nodeId, nodeDofs } from "./web/src/mesh.js";
import { runTopologyOptimization } from "./web/src/optimize.js";
import { findConnectedPath } from "./web/src/connectivity.js";
import { kgToVolumeFraction, kgToNewtons, maxForceFromArea, BRIDGE_SPAN_M, BRIDGE_RISE_M, MAX_MATERIAL_KG } from "./web/src/units.js";

const numElemX = 60;
const numElemY = 30;
const cellWidthM = Math.min(BRIDGE_SPAN_M / numElemX, BRIDGE_RISE_M / numElemY);
const DENSITY_THRESHOLD = 0.5;

const bottomLeft = nodeId(0, 0, numElemX);
const bottomRight = nodeId(numElemX, 0, numElemX);
const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

function closestCell(cells, targetElx, targetEly) {
  let best = null;
  let bestDist = Infinity;
  for (const cell of cells) {
    const dist = (cell.elx - targetElx) ** 2 + (cell.ely - targetEly) ** 2;
    if (dist < bestDist) {
      bestDist = dist;
      best = cell;
    }
  }
  return best;
}

// Shape only depends on (loadColumn, materialKg) — computing it once per
// pair and cheaply checking every weightKg against the cached minDensity
// avoids re-running the optimizer 6x for nothing.
function computeMinDensities(loadColumn, materialKg) {
  const volumeFraction = kgToVolumeFraction(materialKg);
  const node = nodeId(loadColumn, numElemY, numElemX);
  const loadDof = nodeDofs(node)[1];
  const { densities } = runTopologyOptimization(numElemX, numElemY, fixedDofs, [[loadDof, -1]], { volumeFraction });

  const isSolid = (elx, ely) =>
    elx >= 0 && elx < numElemX && ely >= 0 && ely < numElemY && densities[ely][elx] > DENSITY_THRESHOLD;

  function strutStatsFor(columnFilter, supportElx) {
    const cells = [];
    for (let ely = 0; ely < numElemY; ely++) {
      for (let elx = 0; elx < numElemX; elx++) {
        if (!isSolid(elx, ely) || !columnFilter(elx)) continue;
        cells.push({ elx, ely });
      }
    }
    if (cells.length === 0) return null;
    const bottomCell = closestCell(cells, supportElx, 0);
    const topCell = closestCell(cells, loadColumn, numElemY);
    const connection = findConnectedPath(densities, DENSITY_THRESHOLD, bottomCell, topCell);
    if (!connection) return null;

    const totalDensitySum = cells.reduce((sum, cell) => sum + densities[cell.ely][cell.elx], 0);
    const gridLength = Math.hypot(topCell.elx - bottomCell.elx, topCell.ely - bottomCell.ely);
    return { totalDensitySum, gridLength };
  }

  return {
    left: strutStatsFor((elx) => elx <= loadColumn, 0),
    right: strutStatsFor((elx) => elx > loadColumn, numElemX),
  };
}

function outcome(stats, weightKg) {
  if (stats.left === null || stats.right === null) return "disconnected";
  const appliedForce = kgToNewtons(weightKg);
  const holds =
    appliedForce <= maxForceFromArea(stats.left.totalDensitySum, cellWidthM, stats.left.gridLength) &&
    appliedForce <= maxForceFromArea(stats.right.totalDensitySum, cellWidthM, stats.right.gridLength);
  return holds ? "holds " : "breaks";
}

console.log(`MAX_MATERIAL_KG = ${MAX_MATERIAL_KG}`);

const positions = {
  center: Math.round(numElemX / 2),
  offCenter: Math.round(numElemX * 0.3),
  nearSupport: Math.round(numElemX * 0.1),
};

const weights = [5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000, 80000, 100000, 120000, 150000];

for (const materialKg of [250, 400, 620, 900, 1200]) {
  const minDensitiesByPosition = {};
  for (const [name, col] of Object.entries(positions)) {
    minDensitiesByPosition[name] = computeMinDensities(col, materialKg);
  }
  for (const weightKg of weights) {
    const results = Object.entries(positions).map(
      ([name]) => `${name}=${outcome(minDensitiesByPosition[name], weightKg)}`
    );
    console.log(`material=${materialKg}kg weight=${weightKg}kg -> ${results.join(", ")}`);
  }
}
