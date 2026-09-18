// Draws the density grid as carved stone onto the (transparent) structure
// canvas: each cell is stone tinted by density, with a little per-cell
// variation and a chiseled edge where it borders empty space, under a
// moonlit top-to-bottom gradient. Row 0 is the bottom of the domain (this
// project's y-up convention), but canvas y grows downward, so rows are
// flipped when drawing.
import { STONE_RGB } from "./theme.js";
import { DOMAIN } from "./scene.js";

const SOLID = 0.5;

function cellNoise(elx, ely) {
  const h = Math.sin(elx * 12.9898 + ely * 78.233) * 43758.5453;
  return h - Math.floor(h);
}

export function renderDensities(ctx, densities) {
  const numElemY = densities.length;
  const numElemX = densities[0].length;
  const cellWidth = DOMAIN.w / numElemX;
  const cellHeight = DOMAIN.h / numElemY;
  const isSolid = (elx, ely) =>
    elx >= 0 && elx < numElemX && ely >= 0 && ely < numElemY && densities[ely][elx] > SOLID;

  ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      const density = densities[ely][elx];
      if (density < 0.02) continue;

      const x = DOMAIN.x + elx * cellWidth;
      const y = DOMAIN.y + (numElemY - 1 - ely) * cellHeight;
      const tint = 0.975 + 0.05 * cellNoise(elx, ely);
      ctx.globalAlpha = density;
      ctx.fillStyle = `rgb(${Math.round(STONE_RGB[0] * tint)},${Math.round(STONE_RGB[1] * tint)},${Math.round(STONE_RGB[2] * tint)})`;
      // Pad each cell by 1px so adjacent cells don't leave hairline gaps
      // from floating-point rounding.
      ctx.fillRect(x, y, cellWidth + 1, cellHeight + 1);
    }
  }
  ctx.globalAlpha = 1;

  // Chiseled edges: a lit line where solid stone borders empty space above
  // or to the left, a shaded one below or to the right.
  const edge = Math.max(2, cellHeight * 0.14);
  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      if (!isSolid(elx, ely)) continue;
      const x = DOMAIN.x + elx * cellWidth;
      const y = DOMAIN.y + (numElemY - 1 - ely) * cellHeight;
      if (!isSolid(elx, ely + 1)) {
        ctx.fillStyle = "rgba(255,248,225,0.5)";
        ctx.fillRect(x, y, cellWidth + 1, edge);
      }
      if (!isSolid(elx - 1, ely)) {
        ctx.fillStyle = "rgba(255,248,225,0.25)";
        ctx.fillRect(x, y, edge, cellHeight + 1);
      }
      if (!isSolid(elx, ely - 1)) {
        ctx.fillStyle = "rgba(0,0,0,0.38)";
        ctx.fillRect(x, y + cellHeight - edge, cellWidth + 1, edge + 1);
      }
      if (!isSolid(elx + 1, ely)) {
        ctx.fillStyle = "rgba(0,0,0,0.25)";
        ctx.fillRect(x + cellWidth - edge, y, edge + 1, cellHeight + 1);
      }
    }
  }

  // Moonlit gradient over the stone only (source-atop paints just where
  // pixels already exist): lighter toward the top, darker toward the base.
  ctx.save();
  ctx.globalCompositeOperation = "source-atop";
  const shade = ctx.createLinearGradient(0, DOMAIN.y, 0, DOMAIN.y + DOMAIN.h);
  shade.addColorStop(0, "rgba(255,252,240,0.14)");
  shade.addColorStop(1, "rgba(0,0,0,0.30)");
  ctx.fillStyle = shade;
  ctx.fillRect(DOMAIN.x, DOMAIN.y, DOMAIN.w, DOMAIN.h);
  ctx.restore();
}
