// Draws a density grid to a canvas — brighter/whiter where material is
// solid, dark where it's empty. Row 0 is the bottom of the domain (this
// project's y-up convention), but canvas y grows downward, so rows are
// flipped when drawing.
export function renderDensities(ctx, densities, width, height) {
  const numElemY = densities.length;
  const numElemX = densities[0].length;
  const cellWidth = width / numElemX;
  const cellHeight = height / numElemY;

  ctx.fillStyle = "#0b1320";
  ctx.fillRect(0, 0, width, height);

  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      const density = densities[ely][elx];
      if (density < 0.02) continue;

      const canvasRow = numElemY - 1 - ely;
      const brightness = Math.round(density * 255);
      ctx.fillStyle = `rgb(${brightness},${brightness},${brightness})`;
      // Pad each cell by 1px so adjacent cells don't leave hairline gaps
      // from floating-point rounding.
      ctx.fillRect(elx * cellWidth, canvasRow * cellHeight, cellWidth + 1, cellHeight + 1);
    }
  }
}
