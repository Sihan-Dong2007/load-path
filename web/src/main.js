import { nodeId, nodeDofs } from "./mesh.js";
import { iterateTopologyOptimization } from "./optimize.js";
import { renderDensities, drawGround } from "./render.js";
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

// The structural domain only fills the upper part of the canvas — the
// bottom strip is a riverbank/canyon floor, painted here and real (as a
// static Matter body) during the collapse scene, so debris that breaks
// free has somewhere to visibly land instead of falling off-canvas.
const GROUND_BAND_PX = 80;
const domainHeight = canvas.height - GROUND_BAND_PX;
const materialSlider = document.getElementById("material-slider");
const materialLabel = document.getElementById("material-label");
const weightSlider = document.getElementById("weight-slider");
const weightLabel = document.getElementById("weight-label");
const messageEl = document.getElementById("message");
const handle = document.getElementById("load-handle");

function getMaterialKg() {
  return Number(materialSlider.value);
}

// A real stone bridge this size turns out to be enormously strong — the
// sweep in sweep.js found the interesting range (some positions hold,
// some don't) spans roughly 10^5 to 10^8 kg, not a range a linear slider
// can usefully cover. The slider itself stays a plain 0-1000 range;
// this maps that onto the real exponent range.
const WEIGHT_EXPONENT_MIN = 5; // 10^5 kg = 100,000 kg
const WEIGHT_EXPONENT_MAX = 8; // 10^8 kg = 100,000,000 kg
const WEIGHT_SLIDER_MAX = 1000;

function getWeightKg() {
  const t = Number(weightSlider.value) / WEIGHT_SLIDER_MAX;
  const exponent = WEIGHT_EXPONENT_MIN + (WEIGHT_EXPONENT_MAX - WEIGHT_EXPONENT_MIN) * t;
  return Math.round(10 ** exponent);
}

// Numbers in the hundred-thousands to hundred-millions don't mean much on
// their own — anchoring to a real, visualizable object's weight (a fully
// loaded Boeing 747, ~400,000kg) makes them legible.
const BOEING_747_KG = 400000;
function formatWeightKg(kg) {
  const planes = kg / BOEING_747_KG;
  const planesText = planes < 10 ? planes.toFixed(1) : Math.round(planes).toLocaleString();
  return `${Math.round(kg).toLocaleString()} kg (≈ ${planesText} × a loaded Boeing 747)`;
}

function setMessage(text) {
  messageEl.textContent = text;
}

function updateLabels() {
  materialLabel.textContent = `${materialSlider.value} kg`;
  weightLabel.textContent = formatWeightKg(getWeightKg());
}
updateLabels();

// Each full cycle (optimize, then test) gets a token. Starting a new one —
// from a fresh drop or a slider change — makes any in-flight animation or
// physics scene notice it's stale and stop touching the canvas, instead of
// two runs racing for it.
let currentRunToken = 0;
let activeScene = null; // the physics scene from the current/most recent test, if any
let lastLoadColumn = null; // set on the first drop; nothing runs before that
let lastDensities = null; // the last converged shape and its displacement
let lastU = null; // field, cached so changing just the test weight
let lastMaterialKg = null; // doesn't re-grow the structure

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

    renderDensities(ctx, value.densities, canvas.width, domainHeight);
    drawGround(ctx, canvas.width, domainHeight, canvas.height);

    if (!value.converged) {
      requestAnimationFrame(step);
    } else {
      startCollapseTest(token, value.densities, value.u, loadColumn, materialKg);
    }
  }

  requestAnimationFrame(step);
}

function startCollapseTest(token, densities, u, loadColumn, materialKg) {
  if (token !== currentRunToken) return;

  lastDensities = densities;
  lastU = u;
  lastMaterialKg = materialKg;

  const testWeightKg = getWeightKg();
  const scene = buildCollapseScene({ numElemX, numElemY, densities, u, loadColumn, testWeightKg, canvas, domainHeight });
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
  startCollapseTest(token, lastDensities, lastU, lastLoadColumn, lastMaterialKg);
}

// Idle until the first drop — a plain solid block, nothing computed yet.
// Dropping the weight is what starts anything at all.
const solidBlock = Array.from({ length: numElemY }, () => new Array(numElemX).fill(1));
renderDensities(ctx, solidBlock, canvas.width, domainHeight);
drawGround(ctx, canvas.width, domainHeight, canvas.height);
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
