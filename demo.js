// Throwaway script to sanity-check the optimizer on a small mesh before
// worrying about performance or a real renderer. Two bottom corners pinned,
// one downward load at top-center — the classic simplest bridge/arch setup.
import { nodeId, nodeDofs } from "./web/src/mesh.js";
import { runTopologyOptimization } from "./web/src/optimize.js";
import { densitiesToAscii } from "./web/src/visualize.js";

const numElemX = Number(process.argv[2]) || 20;
const numElemY = Number(process.argv[3]) || 10;

const bottomLeft = nodeId(0, 0, numElemX);
const bottomRight = nodeId(numElemX, 0, numElemX);
const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

const topCenter = nodeId(numElemX / 2, numElemY, numElemX);
const [, topCenterY] = nodeDofs(topCenter);
const loads = [[topCenterY, -1]];

console.time("optimize");
const { densities, iterations, converged } = runTopologyOptimization(
  numElemX,
  numElemY,
  fixedDofs,
  loads,
  { volumeFraction: 0.4 }
);
console.timeEnd("optimize");

console.log(`converged=${converged} after ${iterations} iterations\n`);
console.log(densitiesToAscii(densities));
