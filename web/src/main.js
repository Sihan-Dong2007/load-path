import { nodeId, nodeDofs } from "./mesh.js";
import { iterateTopologyOptimization } from "./optimize.js";
import { renderDensities } from "./render.js";

// Same fixed scenario as demo.js for now — two bottom corners pinned, one
// downward load at top-center — until phone interaction can supply real
// load/support positions.
const numElemX = 60;
const numElemY = 30;
const volumeFraction = 0.4;

const bottomLeft = nodeId(0, 0, numElemX);
const bottomRight = nodeId(numElemX, 0, numElemX);
const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

const topCenter = nodeId(numElemX / 2, numElemY, numElemX);
const [, topCenterY] = nodeDofs(topCenter);
const loads = [[topCenterY, -1]];

const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");

const initialDensities = Array.from({ length: numElemY }, () => new Array(numElemX).fill(volumeFraction));
renderDensities(ctx, initialDensities, canvas.width, canvas.height);

const iterator = iterateTopologyOptimization(numElemX, numElemY, fixedDofs, loads, { volumeFraction });

function step() {
  const { value, done } = iterator.next();
  if (done) return;

  renderDensities(ctx, value.densities, canvas.width, canvas.height);

  if (!value.converged) {
    requestAnimationFrame(step);
  }
}

requestAnimationFrame(step);
