import { nodeId, nodeDofs } from "./mesh.js";
import { runTopologyOptimization } from "./optimize.js";

const { Engine, Render, Runner, Bodies, Body, Composite, Constraint, Events, Vector } = window.Matter;

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
// load point — a triangle. First attempt modeled every grid cell as its
// own body welded into a ~48-part Matter.js compound; that produced wildly
// wrong part positions and exploding angular velocities that never got
// fully explained — a real limitation hit while building this, not
// something to paper over. Simplifying to ONE plain rectangle per side
// (its two ends placed exactly at the real support and apex positions,
// rotated to match) sidesteps whatever was going wrong with compound
// bodies entirely, and is physically equivalent for this purpose: each
// side is a single rigid beam either way.
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

  const body = Bodies.rectangle(centerX, centerY, length, cellSize * 1.4);
  Body.setAngle(body, angle);
  // The two beams touch near the apex; without this Matter.js's normal
  // collision response would push them apart right where the tie is
  // trying to hold them together.
  body.collisionFilter.group = -1;
  Composite.add(world, body);

  const halfLength = length / 2;
  return {
    body,
    supportWorld: bottomCell,
    apexWorld: topCell,
    bottomOffset: { x: -halfLength, y: 0 }, // local, unrotated — Matter.js rotates this by body.angle itself
    topOffset: { x: halfLength, y: 0 },
  };
}

const midColumn = numElemX / 2;
const left = buildHalf((elx) => elx <= midColumn, 0);
const right = buildHalf((elx) => elx > midColumn, numElemX);

// Pin each beam's foot to a fixed point in space — a real support, but one
// that (like a real pinned support) still lets the beam rotate around it.
function pinToGround(half) {
  Composite.add(
    world,
    Constraint.create({
      bodyA: half.body,
      pointA: half.bottomOffset,
      pointB: half.supportWorld,
      length: 0,
      stiffness: 1,
    })
  );
}
pinToGround(left);
pinToGround(right);

function worldPoint(half, offset) {
  return Vector.add(half.body.position, Vector.rotate(offset, half.body.angle));
}

// The breakable joint: a single tie between the beams' top ends. Two
// ground pins (4 DOF removed) + one apex tie (2 DOF removed) = 6, exactly
// matching the 6 DOF of two 2D rigid bodies — a perfectly-determined rigid
// triangle, not over- or under-constrained. An earlier version used two
// ties at the apex to try to resist relative rotation too, but that
// over-constrained the system (8 DOF worth of constraints on 6 DOF) and
// made the solver unstable — it blew up within the very first step even
// with a clean, verified-correct initial state. One tie is both simpler
// and the numerically well-behaved choice.
function makeTie(offsetName) {
  const a = worldPoint(left, left[offsetName]);
  const b = worldPoint(right, right[offsetName]);
  return Constraint.create({
    bodyA: left.body,
    pointA: left[offsetName],
    bodyB: right.body,
    pointB: right[offsetName],
    length: Vector.magnitude(Vector.sub(b, a)),
    stiffness: 1,
  });
}

const apexTies = [makeTie("topOffset")];
Composite.add(world, apexTies);

// Any freshly-built rigid joint takes the solver a few frames to settle
// into equilibrium — gravity is applied all at once, and correcting the
// resulting constraint violation happens gradually over several
// iterations of the *simulation*, not within one frame. A first-frame
// strain spike (measured over 200x rest length in testing, settling to
// under 5% within ~15 frames) is normal solver behavior, not a sign of
// overload — checking for breaks before that settling finishes broke the
// joint on frame 0 every time, before the structure ever got a chance to
// prove it could hold on its own.
const SETTLE_FRAMES = 40;
const BREAK_STRAIN = 0.3;
let frameCount = 0;
Events.on(engine, "afterUpdate", () => {
  frameCount++;
  for (const tie of apexTies) {
    if (tie.broken) continue;
    if (frameCount <= SETTLE_FRAMES) continue;
    const a = worldPoint(left, tie.pointA);
    const b = worldPoint(right, tie.pointB);
    const currentLength = Vector.magnitude(Vector.sub(b, a));
    const strain = Math.abs(currentLength - tie.length) / cellSize;
    if (strain > BREAK_STRAIN) {
      Composite.remove(world, tie);
      tie.broken = true;
    }
  }
});

// Ground + walls so debris settles instead of falling forever.
const ground = Bodies.rectangle(canvasWidth / 2, canvasHeight - 10, canvasWidth, 20, { isStatic: true });
const leftWall = Bodies.rectangle(-10, canvasHeight / 2, 20, canvasHeight, { isStatic: true });
const rightWall = Bodies.rectangle(canvasWidth + 10, canvasHeight / 2, 20, canvasHeight, { isStatic: true });
Composite.add(world, [ground, leftWall, rightWall]);

// The test weight, dropped from above the same load point used to
// generate the structure.
const testBall = Bodies.circle(physicsX(midColumn), 20, 18, { density: 1, restitution: 0.1 });
Composite.add(world, testBall);

const render = Render.create({
  element: document.body,
  engine,
  options: { width: canvasWidth, height: canvasHeight, wireframes: false, background: "#0b1320" },
});
const runner = Runner.create();
Render.run(render);
Runner.run(runner, engine);

window.__debug = { engine, render, runner, testBall, apexTies, left, right, world, worldPoint };
