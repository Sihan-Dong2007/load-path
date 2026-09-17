import { nodeId, nodeDofs } from "./mesh.js";
import { runTopologyOptimization } from "./optimize.js";
import { findConnectedPath } from "./connectivity.js";

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
//
// Before building anything physical, check that the support and apex are
// actually connected by real material (findConnectedPath) — at a low
// enough volume fraction the optimizer can produce a design with no
// continuous path at all, and drawing a beam straight through that gap
// would be showing something the optimizer never actually built. Returns
// null in that case instead of a half-built lie.
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
  const topCell = closestCell(cells, numElemX / 2, numElemY);

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

  return { body, minDensity: connection.minDensity };
}

const midColumn = numElemX / 2;
const left = buildHalf((elx) => elx <= midColumn, 0);
const right = buildHalf((elx) => elx > midColumn, numElemX);

if (!left || !right) {
  // No continuous path at this volume fraction / load position — the
  // real UI will turn this into "this amount of material can't even form
  // a structure here," but for this prototype, say so loudly and stop
  // rather than building physics on top of a strut that doesn't exist.
  throw new Error(
    `No connected path from support to apex on ${!left ? "the left" : ""}${!left && !right ? " and " : ""}${!right ? "the right" : ""} side — volumeFraction ${volumeFraction} is too low for this load position.`
  );
}

// Ground + walls so debris settles instead of falling forever.
const ground = Bodies.rectangle(canvasWidth / 2, canvasHeight - 10, canvasWidth, 20, { isStatic: true });
const leftWall = Bodies.rectangle(-10, canvasHeight / 2, 20, canvasHeight, { isStatic: true });
const rightWall = Bodies.rectangle(canvasWidth + 10, canvasHeight / 2, 20, canvasHeight, { isStatic: true });
Composite.add(world, [ground, leftWall, rightWall]);

// The test weight, dropped from above the same load point used to
// generate the structure.
const testBall = Bodies.circle(physicsX(midColumn), 20, 18, { density: 0.05, friction: 0.9, restitution: 0.1 });
Composite.add(world, testBall);

// Breaking is a collision question, not a constraint-strain question: on
// impact, momentum (mass x speed) stands in for how hard the ball hit.
// Above that beam's break threshold, it stops being static and starts
// falling like any other dynamic body from that point on.
//
// The threshold isn't the same fixed number for every beam — it's scaled
// by that beam's own minDensity (the weakest point found along its actual
// connected path). A beam the optimizer built thin should be easier to
// break than one it built thick, or the material budget the user chose
// would have no effect on the outcome at all — which was exactly the gap
// this replaces.
const BASE_BREAK_MOMENTUM = 60;
Events.on(engine, "collisionStart", (event) => {
  for (const pair of event.pairs) {
    const bodies = [pair.bodyA, pair.bodyB];
    const half = [left, right].find((h) => bodies.includes(h.body) && h.body.isStatic);
    const ball = bodies.find((b) => b === testBall);
    if (!half || !ball) continue;

    const momentum = ball.mass * Vector.magnitude(ball.velocity);
    const breakMomentum = BASE_BREAK_MOMENTUM * half.minDensity;
    if (momentum > breakMomentum) {
      Body.setStatic(half.body, false);
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
