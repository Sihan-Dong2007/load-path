// Re-verifies every claim every lesson makes, against the real model: each
// (stone, spot, weight) -> holds / breaks must come out as stated, with at
// least a 1.15x margin so a lesson isn't resting on a coin-flip. Run this after
// changing any physics constant; a stale lesson fails here instead of on the wall.
//
//   node check-lessons.js
import { nodeId, nodeDofs } from "./web/src/mesh.js";
import { runTopologyOptimization } from "./web/src/optimize.js";
import { loadLimit } from "./web/src/failure.js";
import { kgToVolumeFraction, WEIGHT_EXPONENT_MIN, WEIGHT_EXPONENT_MAX } from "./web/src/units.js";
import { LESSONS, weightToSliderValue } from "./web/src/lessons.js";

const nx = 60;
const ny = 30;
const MARGIN = 1.15;
const fixed = [...nodeDofs(nodeId(0, 0, nx)), ...nodeDofs(nodeId(nx, 0, nx))];
const cache = new Map();

function capacity(material, column) {
  const key = `${material}/${column}`;
  if (!cache.has(key)) {
    const [, dof] = nodeDofs(nodeId(column, ny, nx));
    const { densities, u } = runTopologyOptimization(nx, ny, fixed, [[dof, -1]], { volumeFraction: kgToVolumeFraction(material) });
    cache.set(key, loadLimit({ numElemX: nx, numElemY: ny, densities, u, loadColumn: column }).kg);
  }
  return cache.get(key);
}

let failures = 0;
for (const lesson of LESSONS) {
  // The slider can't hit the stated weight exactly (it has 1001 steps); the weight the page will
  // really use is the one the slider value maps to.
  const slider = weightToSliderValue(lesson.weightKg, WEIGHT_EXPONENT_MIN, WEIGHT_EXPONENT_MAX);
  const actualKg = Math.round(10 ** (WEIGHT_EXPONENT_MIN + ((WEIGHT_EXPONENT_MAX - WEIGHT_EXPONENT_MIN) * slider) / 1000));
  console.log(`${lesson.title} (slider ${slider} -> ${actualKg.toLocaleString()} kg)`);
  for (const claim of lesson.claims) {
    const weight = claim === lesson.claims[0] ? actualKg : claim.weightKg;
    const cap = capacity(claim.material, claim.column);
    const holds = cap >= weight;
    const margin = holds ? cap / weight : weight / cap;
    const ok = (claim.expect === "holds") === holds && margin >= MARGIN;
    if (!ok) failures++;
    console.log(`  ${ok ? "ok  " : "FAIL"} ${claim.material} kg at column ${claim.column}, ${weight.toLocaleString()} kg: capacity ${Math.round(cap).toLocaleString()} kg -> ${holds ? "holds" : "breaks"} (expected ${claim.expect}, margin ${margin.toFixed(2)}x)`);
  }
}
console.log(failures === 0 ? "\nAll lesson claims verified." : `\n${failures} lesson claim(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
