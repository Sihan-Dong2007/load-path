import { nodeId, nodeDofs } from "./mesh.js";
import { iterateTopologyOptimization } from "./optimize.js";
import { renderDensities } from "./render.js";

// Supports stay fixed at the two bottom corners for now — only the load
// point is interactive. Movable supports can come later.
const numElemX = 60;
const numElemY = 30;
const volumeFraction = 0.4;

const bottomLeft = nodeId(0, 0, numElemX);
const bottomRight = nodeId(numElemX, 0, numElemX);
const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");

// Each drop starts a fresh run. `currentRunToken` lets an in-progress
// animation notice it's been superseded and stop drawing, instead of
// racing a newer run for control of the canvas.
let currentRunToken = 0;

function runAndAnimate(loadDof) {
  const token = ++currentRunToken;
  const loads = [[loadDof, -1]];
  const iterator = iterateTopologyOptimization(numElemX, numElemY, fixedDofs, loads, { volumeFraction });

  function step() {
    if (token !== currentRunToken) return;

    const { value, done } = iterator.next();
    if (done) return;

    renderDensities(ctx, value.densities, canvas.width, canvas.height);

    if (!value.converged) {
      requestAnimationFrame(step);
    }
  }

  requestAnimationFrame(step);
}

// The default, shown before any interaction: a straight-down load at
// top-center — the same scenario as demo.js.
const defaultTopCenter = nodeId(numElemX / 2, numElemY, numElemX);
runAndAnimate(nodeDofs(defaultTopCenter)[1]);

// --- Drag-and-drop: dropping the weight onto the canvas sets a new load
// point there and reruns the optimization from scratch. ---
const handle = document.getElementById("load-handle");

// Converts a point in page coordinates to the vertical-displacement DOF of
// the nearest node along the TOP edge, or null if the point isn't over the
// canvas at all. Only the horizontal position is used — a dropped weight
// always rests on top of the structure, never inside it. That's both more
// physically sensible (a load embedded inside the material rather than
// resting on its surface isn't a scenario this represents) and sidesteps a
// real FEM artifact: a point load on an interior node is a stress
// singularity, which showed up as a visible kink in testing.
function clientPointToLoadDof(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const relX = (clientX - rect.left) / rect.width;
  const relY = (clientY - rect.top) / rect.height;
  if (relX < 0 || relX > 1 || relY < 0 || relY > 1) return null;

  const col = Math.round(relX * numElemX);
  const node = nodeId(col, numElemY, numElemX); // numElemY = the top row

  return nodeDofs(node)[1]; // the vertical DOF — a dropped weight pulls straight down
}

let dragOffsetX = 0;
let dragOffsetY = 0;

handle.addEventListener("pointerdown", (event) => {
  handle.setPointerCapture(event.pointerId);
  const rect = handle.getBoundingClientRect();
  dragOffsetX = event.clientX - rect.left;
  dragOffsetY = event.clientY - rect.top;
  handle.classList.add("dragging");
});

handle.addEventListener("pointermove", (event) => {
  if (!handle.classList.contains("dragging")) return;
  handle.style.left = `${event.clientX - dragOffsetX}px`;
  handle.style.top = `${event.clientY - dragOffsetY}px`;
});

handle.addEventListener("pointerup", (event) => {
  handle.classList.remove("dragging");
  handle.style.left = "";
  handle.style.top = "";

  const loadDof = clientPointToLoadDof(event.clientX, event.clientY);
  if (loadDof !== null) {
    runAndAnimate(loadDof);
  }
});
