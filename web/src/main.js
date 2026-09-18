import { nodeId, nodeDofs } from "./mesh.js";
import { iterateTopologyOptimization } from "./optimize.js";
import { renderDensities } from "./render.js";
import { buildCollapseScene } from "./collapse.js";
import { kgToVolumeFraction } from "./units.js";

// Supports stay fixed at the two bottom corners — only the load point and
// the material budget are interactive.
const numElemX = 60;
const numElemY = 30;

const bottomLeft = nodeId(0, 0, numElemX);
const bottomRight = nodeId(numElemX, 0, numElemX);
const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");
const materialSlider = document.getElementById("material-slider");
const materialLabel = document.getElementById("material-label");
const weightSlider = document.getElementById("weight-slider");
const weightLabel = document.getElementById("weight-label");
const messageEl = document.getElementById("message");
const handle = document.getElementById("load-handle");

function getMaterialKg() {
  return Number(materialSlider.value);
}

function getWeightKg() {
  return Number(weightSlider.value);
}

function setMessage(text) {
  messageEl.textContent = text;
}

function updateLabels() {
  materialLabel.textContent = `${materialSlider.value} kg`;
  weightLabel.textContent = `${weightSlider.value} kg`;
}
updateLabels();

// Each full cycle (optimize, then test) gets a token. Starting a new one —
// from a fresh drop or a slider change — makes any in-flight animation or
// physics scene notice it's stale and stop touching the canvas, instead of
// two runs racing for it.
let currentRunToken = 0;
let activeScene = null; // the physics scene from the current/most recent test, if any
let lastLoadColumn = null; // set on the first drop; nothing runs before that
let lastDensities = null; // the last converged shape, cached so changing
let lastMaterialKg = null; // just the test weight doesn't re-grow it

function runFullCycle(loadColumn) {
  const token = ++currentRunToken;
  if (activeScene) {
    activeScene.stop();
    activeScene = null;
  }

  lastLoadColumn = loadColumn;
  const materialKg = getMaterialKg();
  const volumeFraction = kgToVolumeFraction(materialKg);

  const node = nodeId(loadColumn, numElemY, numElemX);
  const loadDof = nodeDofs(node)[1];
  const loads = [[loadDof, -1]];

  setMessage("Growing the structure…");
  const iterator = iterateTopologyOptimization(numElemX, numElemY, fixedDofs, loads, { volumeFraction });

  function step() {
    if (token !== currentRunToken) return;

    const { value, done } = iterator.next();
    if (done) return;

    renderDensities(ctx, value.densities, canvas.width, canvas.height);

    if (!value.converged) {
      requestAnimationFrame(step);
    } else {
      startCollapseTest(token, value.densities, loadColumn, materialKg);
    }
  }

  requestAnimationFrame(step);
}

function startCollapseTest(token, densities, loadColumn, materialKg) {
  if (token !== currentRunToken) return;

  lastDensities = densities;
  lastMaterialKg = materialKg;

  const testWeightKg = getWeightKg();
  const scene = buildCollapseScene({ numElemX, numElemY, densities, loadColumn, testWeightKg, canvas });
  if (!scene.ok) {
    setMessage(`${materialKg}kg of stone can't even form a structure here — try more stone, or a different spot.`);
    return;
  }

  activeScene = scene;
  setMessage("Testing it…");
  scene.dropBall();

  function poll() {
    if (token !== currentRunToken) return; // stop() already called by whoever superseded this run
    if (scene.hasSettled()) {
      if (scene.collapsed()) {
        setMessage(`It broke under ${testWeightKg}kg. Try more stone, or a different spot.`);
      } else {
        setMessage(`It held ${testWeightKg}kg. Try less stone, or a different spot.`);
      }
      return;
    }
    requestAnimationFrame(poll);
  }

  requestAnimationFrame(poll);
}

// Re-runs just the physics test against the shape already grown for the
// current position/material, since changing only the test weight doesn't
// change what shape the optimizer would produce.
function retest() {
  if (lastDensities === null) return;
  const token = ++currentRunToken;
  if (activeScene) {
    activeScene.stop();
    activeScene = null;
  }
  startCollapseTest(token, lastDensities, lastLoadColumn, lastMaterialKg);
}

// Idle until the first drop — a plain solid block, nothing computed yet.
// Dropping the weight is what starts anything at all.
const solidBlock = Array.from({ length: numElemY }, () => new Array(numElemX).fill(1));
renderDensities(ctx, solidBlock, canvas.width, canvas.height);
setMessage("Drag the weight onto the block to begin.");

materialSlider.addEventListener("input", updateLabels);
materialSlider.addEventListener("change", () => {
  updateLabels();
  if (lastLoadColumn !== null) runFullCycle(lastLoadColumn);
});

// The weight only matters once a structure already exists — changing it
// re-tests the SAME converged shape rather than re-growing it, since the
// load position and material budget (what the shape depends on) haven't
// changed.
weightSlider.addEventListener("input", updateLabels);
weightSlider.addEventListener("change", () => {
  updateLabels();
  retest();
});

// --- Drag-and-drop: dropping the weight onto the canvas sets a new load
// column there and reruns the whole cycle from scratch. ---

// Converts a point in page coordinates to the nearest column, or null if
// the point isn't over the canvas at all. Only the horizontal position is
// used — a dropped weight always rests on top of the structure, never
// inside it (see main.js history for why: an interior point load is a
// real FEM stress singularity).
function clientPointToColumn(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const relX = (clientX - rect.left) / rect.width;
  const relY = (clientY - rect.top) / rect.height;
  if (relX < 0 || relX > 1 || relY < 0 || relY > 1) return null;

  return Math.round(relX * numElemX);
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

  const column = clientPointToColumn(event.clientX, event.clientY);
  if (column !== null) {
    runFullCycle(column);
  }
});
