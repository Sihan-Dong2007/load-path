import { findConnectedPath } from "./connectivity.js";
import { BRIDGE_SPAN_M, BRIDGE_RISE_M, kgToNewtons, maxForceFromArea } from "./units.js";

const { Engine, Render, Runner, Bodies, Body, Composite, Vector } = window.Matter;

const DENSITY_THRESHOLD = 0.5;
const SETTLE_SPEED = 0.05;
const SETTLE_FRAMES_REQUIRED = 30;
const MAX_FRAMES = 600; // ~10s safety cap in case something never quite settles

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

// Builds a physics scene testing whether a converged density grid holds
// under a given real-world test weight, drawing onto the SAME canvas the
// optimization animation just used (Matter.Render takes over the canvas
// once this is built — don't draw over it with renderDensities after).
//
// Whether each beam holds is decided analytically before any physics
// runs at all — real applied force (from testWeightKg) against that
// beam's real load capacity (stone's tensile strength x its weakest
// point's real cross-sectional area, from units.js) — not by anything
// Matter.js measures during the simulation. Matter.js's only job here is
// to animate whichever outcome that calculation already determined:
// setting a beam dynamic on impact if it was going to fail, and never
// touching it if it wasn't.
//
// Returns { ok: false } if the support and apex on either side aren't
// actually connected by material at this volume fraction/load position —
// there's nothing to test, so the caller should treat that as an
// automatic failure without spawning any physics. Otherwise returns an
// object with dropBall()/hasSettled()/collapsed()/stop().
export function buildCollapseScene({ numElemX, numElemY, densities, loadColumn, testWeightKg, canvas }) {
  const canvasWidth = canvas.width;
  const canvasHeight = canvas.height;
  const cellWidth = canvasWidth / numElemX;
  const cellHeight = canvasHeight / numElemY;
  const cellWidthM = Math.min(BRIDGE_SPAN_M / numElemX, BRIDGE_RISE_M / numElemY);

  const physicsX = (elx) => elx * cellWidth + cellWidth / 2;
  const physicsY = (ely) => canvasHeight - (ely * cellHeight + cellHeight / 2); // flip: ely up == y down

  const isSolid = (elx, ely) =>
    elx >= 0 && elx < numElemX && ely >= 0 && ely < numElemY && densities[ely][elx] > DENSITY_THRESHOLD;

  const engine = Engine.create();
  const world = engine.world;

  function buildHalf(columnFilter, supportElx) {
    const cells = [];
    for (let ely = 0; ely < numElemY; ely++) {
      for (let elx = 0; elx < numElemX; elx++) {
        if (!isSolid(elx, ely) || !columnFilter(elx)) continue;
        cells.push({ elx, ely, x: physicsX(elx), y: physicsY(ely) });
      }
    }
    if (cells.length === 0) return null;

    const bottomCell = closestCell(cells, supportElx, 0);
    const topCell = closestCell(cells, loadColumn, numElemY);

    const connection = findConnectedPath(
      densities,
      DENSITY_THRESHOLD,
      { elx: bottomCell.elx, ely: bottomCell.ely },
      { elx: topCell.elx, ely: topCell.ely }
    );
    if (!connection) return null;

    const dx = topCell.x - bottomCell.x;
    const dy = topCell.y - bottomCell.y;
    const length = Math.hypot(dx, dy);
    const angle = Math.atan2(dy, dx);
    const centerX = (bottomCell.x + topCell.x) / 2;
    const centerY = (bottomCell.y + topCell.y) / 2;
    const thickness = Math.max(cellWidth, cellHeight) * 1.4;
    const overlap = Math.max(cellWidth, cellHeight);

    const body = Bodies.rectangle(centerX, centerY, length + overlap, thickness, { friction: 0.9 });
    Body.setAngle(body, angle);
    body.collisionFilter.group = -1; // the two beams overlap by design near the apex
    Body.setStatic(body, true);
    Composite.add(world, body);

    // Total density-weighted area this half actually used, and its real
    // length in grid units — together these give an average cross-section
    // (maxForceFromArea) that responds to material budget, unlike a
    // single weakest cell's density (see units.js).
    const totalDensitySum = cells.reduce((sum, cell) => sum + densities[cell.ely][cell.elx], 0);
    const gridLength = Math.hypot(topCell.elx - bottomCell.elx, topCell.ely - bottomCell.ely);

    return { body, totalDensitySum, gridLength };
  }

  const left = buildHalf((elx) => elx <= loadColumn, 0);
  const right = buildHalf((elx) => elx > loadColumn, numElemX);

  if (!left || !right) {
    Engine.clear(engine);
    return { ok: false };
  }

  // The engineering decision, made once, up front — not measured during
  // the simulation.
  const appliedForce = kgToNewtons(testWeightKg);
  const willBreak = {
    left: appliedForce > maxForceFromArea(left.totalDensitySum, cellWidthM, left.gridLength),
    right: appliedForce > maxForceFromArea(right.totalDensitySum, cellWidthM, right.gridLength),
  };

  const ballRadius = Math.max(cellWidth, cellHeight) * 1.2;
  const dropStartY = physicsY(numElemY) - ballRadius * 3;
  // The ball's Matter.js mass only affects how the drop LOOKS (a heavier
  // test weight should visibly fall with more presence) — it has no say
  // in whether anything breaks, since that was already decided above.
  const testBall = Bodies.circle(physicsX(loadColumn), dropStartY, ballRadius, {
    density: 0.002 + testWeightKg * 0.0000003,
    friction: 0.9,
    restitution: 0.1,
    isStatic: true, // starts suspended — "the weight resting where you placed it"
  });
  Composite.add(world, testBall);

  const { Events } = window.Matter;
  Events.on(engine, "collisionStart", (event) => {
    for (const pair of event.pairs) {
      const bodies = [pair.bodyA, pair.bodyB];
      if (!bodies.includes(testBall)) continue;
      if (bodies.includes(left.body) && left.body.isStatic && willBreak.left) Body.setStatic(left.body, false);
      if (bodies.includes(right.body) && right.body.isStatic && willBreak.right) Body.setStatic(right.body, false);
    }
  });

  const render = Render.create({
    canvas,
    engine,
    options: { width: canvasWidth, height: canvasHeight, wireframes: false, background: "#0b1320" },
  });
  const runner = Runner.create();

  let settledFrames = 0;
  let totalFrames = 0;

  return {
    ok: true,
    dropBall() {
      Body.setStatic(testBall, false);
      Render.run(render);
      Runner.run(runner, engine);
    },
    hasSettled() {
      totalFrames++;
      if (totalFrames > MAX_FRAMES) return true;

      const speeds = [Vector.magnitude(testBall.velocity)];
      if (!left.body.isStatic) speeds.push(Vector.magnitude(left.body.velocity));
      if (!right.body.isStatic) speeds.push(Vector.magnitude(right.body.velocity));

      settledFrames = Math.max(...speeds) < SETTLE_SPEED ? settledFrames + 1 : 0;
      return settledFrames > SETTLE_FRAMES_REQUIRED;
    },
    collapsed() {
      return !left.body.isStatic || !right.body.isStatic;
    },
    stop() {
      Runner.stop(runner);
      Render.stop(render);
      Composite.clear(world, false);
      Engine.clear(engine);
    },
  };
}
