import { nodeId, nodeDofs } from "./mesh.js";
import { runTopologyOptimization } from "./optimize.js";

const { Engine, Render, Runner, Bodies, Body, Composite, Events, Vector } = window.Matter;

// Reuse the same scenario as demo.js — real optimizer output, not a
// hand-drawn fake structure.
const numElemX = 20;
const numElemY = 10;
const volumeFraction = 0.4;

const bottomLeft = nodeId(0, 0, numElemX);
const bottomRight = nodeId(numElemX, 0, numElemX);
const fixedDofs = [...nodeDofs(bottomLeft), ...nodeDofs(bottomRight)];

const topCenter = nodeId(numElemX / 2, numElemY, numElemX);
const [, topCenterY] = nodeDofs(topCenter);
const loads = [[topCenterY, -1]];

const { densities } = runTopologyOptimization(numElemX, numElemY, fixedDofs, loads, { volumeFraction });

// --- Map the density grid into physics space ---
const cellSize = 16;
const canvasWidth = 640;
const canvasHeight = 520;
const offsetX = (canvasWidth - numElemX * cellSize) / 2;
const domainBottomY = 340; // physics y of the design domain's bottom edge (ely = 0)

const physicsX = (elx) => offsetX + elx * cellSize + cellSize / 2;
const physicsY = (ely) => domainBottomY - ely * cellSize - cellSize / 2; // flip: ely up == y down

const DENSITY_THRESHOLD = 0.5;
const isSolid = (elx, ely) =>
  elx >= 0 && elx < numElemX && ely >= 0 && ely < numElemY && densities[ely][elx] > DENSITY_THRESHOLD;

const engine = Engine.create();
const world = engine.world;

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

// This converged shape is two diagonal struts meeting at the top-center
// load point — a triangle. Each side is one rigid beam (a single rotated
// rectangle).
//
// Earlier versions tried to hold the beams up with Matter.js constraints
// (a single pin per foot, then two points per foot for a rigid weld) and
// kept hitting the same wall: any pin that lets a beam rotate makes
// "apex up" merely an UNSTABLE equilibrium (like a pencil on its tip),
// which the normal first-frame solver settling was enough to tip over;
// welding it down instead over-constrained the system and made the
// solver unstable outright. Both are real, documented problems with
// iterative constraint solvers, not something worth continuing to fight.
//
// The fix: don't use a constraint at all. Build each beam as isStatic —
// truly, unconditionally fixed in place, no settling, no equilibrium to
// tip over — and only make it dynamic (so it starts falling under
// gravity) at the moment a hard enough impact says it should.
function buildHalf(columnFilter, supportElx) {
  const cells = [];
  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      if (!isSolid(elx, ely) || !columnFilter(elx)) continue;
      cells.push({ elx, ely, x: physicsX(elx), y: physicsY(ely) });
    }
  }

  const bottomCell = closestCell(cells, supportElx, 0);
  const topCell = closestCell(cells, numElemX / 2, numElemY);

  const dx = topCell.x - bottomCell.x;
  const dy = topCell.y - bottomCell.y;
  const length = Math.hypot(dx, dy);
  const angle = Math.atan2(dy, dx);
  const centerX = (bottomCell.x + topCell.x) / 2;
  const centerY = (bottomCell.y + topCell.y) / 2;

  // A bit longer than the true support-to-apex distance so the two
  // beams' tips overlap at the top, giving a dropped weight a solid
  // surface to land on.
  // High friction so a dropped weight settles where it lands instead of
  // sliding down the slope and hitting things again — which caused
  // several small, harmless-looking collisions to add up to a break that
  // the first impact alone didn't cause.
  const body = Bodies.rectangle(centerX, centerY, length + cellSize, cellSize * 1.4, { friction: 0.9 });
  Body.setAngle(body, angle);
  Body.setStatic(body, true);
  Composite.add(world, body);

  return { body };
}

const midColumn = numElemX / 2;
const left = buildHalf((elx) => elx <= midColumn, 0);
const right = buildHalf((elx) => elx > midColumn, numElemX);

// Ground + walls so debris settles instead of falling forever.
const ground = Bodies.rectangle(canvasWidth / 2, canvasHeight - 10, canvasWidth, 20, { isStatic: true });
const leftWall = Bodies.rectangle(-10, canvasHeight / 2, 20, canvasHeight, { isStatic: true });
const rightWall = Bodies.rectangle(canvasWidth + 10, canvasHeight / 2, 20, canvasHeight, { isStatic: true });
Composite.add(world, [ground, leftWall, rightWall]);

// The test weight, dropped from above the same load point used to
// generate the structure.
const testBall = Bodies.circle(physicsX(midColumn), 20, 18, { density: 0.05, friction: 0.9, restitution: 0.1 });
Composite.add(world, testBall);

// Breaking is now a collision question, not a constraint-strain question:
// on impact, momentum (mass x speed) standing in for how hard the ball
// hit. Above BREAK_MOMENTUM, whichever beam(s) it hit stop being static
// and start falling like any other dynamic body from that point on.
const BREAK_MOMENTUM = 60;
Events.on(engine, "collisionStart", (event) => {
  for (const pair of event.pairs) {
    const beam = [pair.bodyA, pair.bodyB].find((b) => (b === left.body || b === right.body) && b.isStatic);
    const ball = [pair.bodyA, pair.bodyB].find((b) => b === testBall);
    if (!beam || !ball) continue;

    const momentum = ball.mass * Vector.magnitude(ball.velocity);
    if (momentum > BREAK_MOMENTUM) {
      Body.setStatic(beam, false);
    }
  }
});

const render = Render.create({
  element: document.body,
  engine,
  options: { width: canvasWidth, height: canvasHeight, wireframes: false, background: "#0b1320" },
});
const runner = Runner.create();
Render.run(render);
Runner.run(runner, engine);

window.__debug = { engine, render, runner, testBall, left, right, world };
