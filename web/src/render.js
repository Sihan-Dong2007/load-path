// Draws a density grid to a canvas — a warm stone tint fading in with
// density, so a cell looks more solid the closer it is to full material.
// Row 0 is the bottom of the domain (this project's y-up convention), but
// canvas y grows downward, so rows are flipped when drawing.
import { SKY_COLOR, STONE_COLOR, GROUND_COLOR, GROUND_STROKE } from "./theme.js";

export function renderDensities(ctx, densities, width, height) {
  const numElemY = densities.length;
  const numElemX = densities[0].length;
  const cellWidth = width / numElemX;
  const cellHeight = height / numElemY;

  ctx.fillStyle = SKY_COLOR;
  ctx.fillRect(0, 0, width, height);

  ctx.fillStyle = STONE_COLOR;
  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      const density = densities[ely][elx];
      if (density < 0.02) continue;

      const canvasRow = numElemY - 1 - ely;
      ctx.globalAlpha = density;
      // Pad each cell by 1px so adjacent cells don't leave hairline gaps
      // from floating-point rounding.
      ctx.fillRect(elx * cellWidth, canvasRow * cellHeight, cellWidth + 1, cellHeight + 1);
    }
  }
  ctx.globalAlpha = 1;
}

// The ground band below the structural domain — riverbanks/canyon floor the
// bridge spans. Purely a painted backdrop here; during the collapse scene
// Matter.js draws its own real (collidable) copy at the same position, so
// debris that breaks free has something to land on.
export function drawGround(ctx, width, domainHeight, canvasHeight) {
  ctx.fillStyle = GROUND_COLOR;
  ctx.fillRect(0, domainHeight, width, canvasHeight - domainHeight);
  ctx.strokeStyle = GROUND_STROKE;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, domainHeight);
  ctx.lineTo(width, domainHeight);
  ctx.stroke();
}
