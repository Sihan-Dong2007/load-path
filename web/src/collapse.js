import { evaluateHalves } from "./failure.js";

const { Engine, Render, Runner, Bodies, Body, Composite, Vector } = window.Matter;

const SETTLE_SPEED = 0.05;
const SETTLE_FRAMES_REQUIRED = 30;
const MAX_FRAMES = 600; // ~10s safety cap in case something never quite settles

// Builds a physics scene testing whether a converged density grid holds
// under a given real-world test weight, drawing onto the SAME canvas the
// optimization animation just used (Matter.Render takes over the canvas
// once this is built — don't draw over it with renderDensities after).
//
// Whether each beam holds is decided analytically before any physics
// runs at all, by evaluateHalves() (failure.js) — real stress from the
// FEM solution against stone's real strength, plus a real Euler buckling
// check — not by anything Matter.js measures during the simulation.
// Physics only animates whichever outcome that calculation already
// determined: setting a beam dynamic on impact if it was going to fail,
// and never touching it if it wasn't.
//
// Returns { ok: false } if the support and apex on either side aren't
// actually connected by material at this volume fraction/load position —
// there's nothing to test, so the caller should treat that as an
// automatic failure without spawning any physics. Otherwise returns an
// object with dropBall()/hasSettled()/collapsed()/stop().
export function buildCollapseScene({ numElemX, numElemY, densities, u, loadColumn, testWeightKg, canvas }) {
  const evaluation = evaluateHalves({ numElemX, numElemY, densities, u, loadColumn, testWeightKg });
  if (!evaluation.ok) return { ok: false };

  const canvasWidth = canvas.width;
  const canvasHeight = canvas.height;
  const cellWidth = canvasWidth / numElemX;
  const cellHeight = canvasHeight / numElemY;
  const physicsX = (elx) => elx * cellWidth + cellWidth / 2;
  const physicsY = (ely) => canvasHeight - (ely * cellHeight + cellHeight / 2); // flip: ely up == y down

  const engine = Engine.create();
  const world = engine.world;

  function buildBody(half) {
    const bottomX = physicsX(half.bottomCell.elx);
    const bottomY = physicsY(half.bottomCell.ely);
    const topX = physicsX(half.topCell.elx);
    const topY = physicsY(half.topCell.ely);

    const dx = topX - bottomX;
    const dy = topY - bottomY;
    const length = Math.hypot(dx, dy);
    const angle = Math.atan2(dy, dx);
    const centerX = (bottomX + topX) / 2;
    const centerY = (bottomY + topY) / 2;
    const thickness = Math.max(cellWidth, cellHeight) * 1.4;
    const overlap = Math.max(cellWidth, cellHeight);

    // A bit longer than the true support-to-apex distance so the two
    // beams' tips overlap at the top, giving a dropped weight a solid
    // surface to land on. High friction so it settles where it lands.
    const body = Bodies.rectangle(centerX, centerY, length + overlap, thickness, { friction: 0.9 });
    Body.setAngle(body, angle);
    body.collisionFilter.group = -1; // the two beams overlap by design near the apex
    Body.setStatic(body, true);
    Composite.add(world, body);
    return body;
  }

  const leftBody = buildBody(evaluation.left);
  const rightBody = buildBody(evaluation.right);
  const willBreak = { left: evaluation.left.fails, right: evaluation.right.fails };

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
      if (bodies.includes(leftBody) && leftBody.isStatic && willBreak.left) Body.setStatic(leftBody, false);
      if (bodies.includes(rightBody) && rightBody.isStatic && willBreak.right) Body.setStatic(rightBody, false);
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
      if (!leftBody.isStatic) speeds.push(Vector.magnitude(leftBody.velocity));
      if (!rightBody.isStatic) speeds.push(Vector.magnitude(rightBody.velocity));

      settledFrames = Math.max(...speeds) < SETTLE_SPEED ? settledFrames + 1 : 0;
      return settledFrames > SETTLE_FRAMES_REQUIRED;
    },
    collapsed() {
      return !leftBody.isStatic || !rightBody.isStatic;
    },
    stop() {
      Runner.stop(runner);
      Render.stop(render);
      Composite.clear(world, false);
      Engine.clear(engine);
    },
  };
}
