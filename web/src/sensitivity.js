import { getElementStiffnessMatrix, matVec } from "./fem.js";
import { elementDofs } from "./mesh.js";

// How much the structure's total compliance would change if element e's
// density moved a tiny bit: dc/dx_e = -penal * x_e^(penal-1) * (u_e^T KE u_e).
// Elements carrying a lot of strain energy (load-bearing) get a
// large-magnitude, very negative value; elements doing nothing get a value
// near zero and are safe to remove.
export function computeSensitivities(numElemX, numElemY, densities, u, penal = 3) {
  const KE = getElementStiffnessMatrix();
  const sensitivities = Array.from({ length: numElemY }, () => new Array(numElemX).fill(0));

  for (let ely = 0; ely < numElemY; ely++) {
    for (let elx = 0; elx < numElemX; elx++) {
      const ue = elementDofs(elx, ely, numElemX).map((d) => u[d]);
      const KEue = matVec(KE, ue);
      const energy = ue.reduce((sum, value, i) => sum + value * KEue[i], 0);

      const density = densities[ely][elx];
      sensitivities[ely][elx] = -penal * density ** (penal - 1) * energy;
    }
  }

  return sensitivities;
}
