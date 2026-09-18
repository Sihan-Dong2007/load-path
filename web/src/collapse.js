import { evaluateHalves } from "./failure.js";
import { buildShards, polygonCentroid } from "./fracture.js";
import { createDustSystem } from "./dust.js";
import { playImpact, playBreak } from "./sound.js";
import {
  SKY_COLOR,
  STONE_COLOR,
  STONE_STROKE,
  BROKEN_STONE_COLOR,
  BROKEN_STONE_STROKE,
  GROUND_COLOR,
  GROUND_STROKE,
  WEIGHT_COLOR,
  WEIGHT_STROKE,
} from "./theme.js";

const { Engine, Render, Runner, Bodies, Body, Composite, Vector } = window.Matter;

const SETTLE_SPEED = 0.05;
const SETTLE_FRAMES_REQUIRED = 30;
const MAX_FRAMES = 600; // ~10s safety cap in case something never quite settles

// Every shard body shares this (negative) collision group, meaning members
// never collide with each other — the same trick the old single-beam
// version used at the apex, now needed everywhere two shards touch,
// including neighboring shards within the same half.
const SHARD_GROUP = -1;
const MIN_SHARDS = 3;
const MAX_SHARDS = 7;
const CELLS_PER_SHARD_TARGET = 25;

function shardCountFor(cellCount) {
  return Math.max(MIN_SHARDS, Math.min(MAX_SHARDS, Math.round(cellCount / CELLS_PER_SHARD_TARGET)));
}

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
export function buildCollapseScene({ numElemX, numElemY, densities, u, loadColumn, testWeightKg, canvas, domainHeight }) {
  const evaluation = evaluateHalves({ numElemX, numElemY, densities, u, loadColumn, testWeightKg });
  if (!evaluation.ok) return { ok: false };

  const canvasWidth = canvas.width;
  const canvasHeight = canvas.height;
  const cellWidth = canvasWidth / numElemX;
  const cellHeight = domainHeight / numElemY;
  const physicsX = (elx) => elx * cellWidth + cellWidth / 2;
  const physicsY = (ely) => domainHeight - (ely * cellHeight + cellHeight / 2); // flip: ely up == y down

  const engine = Engine.create();
  const world = engine.world;

  // Riverbank/canyon floor the bridge spans — real (collidable) here, unlike
  // the painted copy drawn during the growth animation, so debris that
  // breaks free has somewhere to land instead of falling off-canvas.
  const groundThickness = canvasHeight - domainHeight;
  Composite.add(
    world,
    Bodies.rectangle(canvasWidth / 2, domainHeight + groundThickness / 2, canvasWidth * 1.4, groundThickness, {
      isStatic: true,
      friction: 0.9,
      render: { fillStyle: GROUND_COLOR, strokeStyle: GROUND_STROKE, lineWidth: 2 },
    })
  );

  // A cell's 4 corners in physics/canvas space — the same rectangle
  // renderDensities() draws for that cell, just without its 1px overlap
  // padding (irrelevant here: a shard's hull only keeps its group's outer
  // boundary, so any interior seams between neighboring cells disappear
  // into the hull anyway).
  function cellCorners(elx, ely) {
    const xLeft = elx * cellWidth;
    const xRight = xLeft + cellWidth;
    const yBottom = domainHeight - ely * cellHeight;
    const yTop = yBottom - cellHeight;
    return [
      { x: xLeft, y: yTop },
      { x: xRight, y: yTop },
      { x: xRight, y: yBottom },
      { x: xLeft, y: yBottom },
    ];
  }

  // One half's real, irregular shape — the same solid cells the strength
  // check evaluated, not an idealized straight beam — split into a few
  // convex shards so a failure can shatter it instead of dropping one
  // rigid slab. All shards start static (the half "resting" as tested);
  // the whole group is set dynamic together the moment this half is
  // determined to fail (see markBroken below), not per-shard, since the
  // physics decision itself is made at the half level, not the shard level.
  function buildShardBodies(half) {
    const shards = buildShards(half.cells, shardCountFor(half.cells.length), (cell) => cellCorners(cell.elx, cell.ely));
    return shards.map(({ vertices }) => {
      const { x: centerX, y: centerY } = polygonCentroid(vertices);
      const body = Bodies.fromVertices(centerX, centerY, [vertices], {
        friction: 0.9,
        isStatic: true,
        collisionFilter: { group: SHARD_GROUP },
        render: { fillStyle: STONE_COLOR, strokeStyle: STONE_STROKE, lineWidth: 1 },
      });
      Composite.add(world, body);
      return body;
    });
  }

  const leftShards = buildShardBodies(evaluation.left);
  const rightShards = buildShardBodies(evaluation.right);
  const willBreak = { left: evaluation.left.fails, right: evaluation.right.fails };
  const triggered = { left: false, right: false };

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
    render: { fillStyle: WEIGHT_COLOR, strokeStyle: WEIGHT_STROKE, lineWidth: 2 },
  });
  Composite.add(world, testBall);

  const render = Render.create({
    canvas,
    engine,
    options: { width: canvasWidth, height: canvasHeight, wireframes: false, background: SKY_COLOR },
  });
  const runner = Runner.create();
  const dust = createDustSystem(render);

  // How hard the drop reads, for both the audio and the dust: derived from
  // the same real-unit exponent range main.js maps the weight slider onto
  // (10^5 - 10^8 kg), so a heavier test weight sounds and looks heavier,
  // not just "breaks or doesn't."
  const impactIntensity = Math.min(1, Math.max(0, (Math.log10(testWeightKg) - 5) / 3));

  // Beyond animating the fall, breaking also recolors every shard in that
  // half to a duller, dustier stone tone and kicks up a dust puff at each
  // one — cues on top of the motion itself that this is the half the
  // physics decided would fail.
  function markBroken(shards) {
    for (const body of shards) {
      Body.setStatic(body, false);
      body.render.fillStyle = BROKEN_STONE_COLOR;
      body.render.strokeStyle = BROKEN_STONE_STROKE;
      const size = (body.bounds.max.x - body.bounds.min.x + (body.bounds.max.y - body.bounds.min.y)) / 4;
      dust.spawnBurst(body.position.x, body.position.y, size);
    }
  }

  // The first ball-structure contact always gets an impact thud, whether or
  // not anything ends up breaking; a half's own break sound plays on top of
  // that, once, the moment it's triggered.
  let hasImpacted = false;

  const { Events } = window.Matter;
  Events.on(engine, "collisionStart", (event) => {
    for (const pair of event.pairs) {
      const bodies = [pair.bodyA, pair.bodyB];
      if (!bodies.includes(testBall)) continue;

      if (!hasImpacted) {
        hasImpacted = true;
        playImpact(impactIntensity);
      }
      if (!triggered.left && willBreak.left && bodies.some((b) => leftShards.includes(b))) {
        triggered.left = true;
        markBroken(leftShards);
        playBreak(impactIntensity);
      }
      if (!triggered.right && willBreak.right && bodies.some((b) => rightShards.includes(b))) {
        triggered.right = true;
        markBroken(rightShards);
        playBreak(impactIntensity);
      }
    }
  });

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

      const dynamicBodies = [testBall, ...leftShards, ...rightShards].filter((b) => !b.isStatic);
      const speeds = dynamicBodies.map((b) => Vector.magnitude(b.velocity));
      settledFrames = Math.max(...speeds) < SETTLE_SPEED ? settledFrames + 1 : 0;
      return settledFrames > SETTLE_FRAMES_REQUIRED;
    },
    collapsed() {
      return triggered.left || triggered.right;
    },
    stop() {
      Runner.stop(runner);
      Render.stop(render);
      Composite.clear(world, false);
      Engine.clear(engine);
    },
  };
}
