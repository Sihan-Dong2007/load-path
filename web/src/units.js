// Physical calibration for the "Chinese classical stone arch bridge"
// theme: turns the abstract volume-fraction / physics-engine numbers
// everywhere else in the project into real kilograms.
//
// Reference structure: a small garden-scale stone footbridge, not a real
// highway span — a real bridge at this aspect ratio would put the material
// budget in the tens of tonnes, which doesn't feel like anything a person
// can relate to on a slider. 2m span x 1m rise x 0.3m wide keeps the numbers
// human-scale while still being an honest "small stone bridge."
export const BRIDGE_SPAN_M = 2;
export const BRIDGE_RISE_M = 1;
export const BRIDGE_DEPTH_M = 0.3; // the bridge's actual width, i.e. the
// out-of-plane thickness the plane-stress FEM model assumes but never
// represents directly.

// Stone/masonry. Real stone is much weaker in tension than compression —
// classical arch bridges are literally shaped to keep the stone in
// compression and avoid tension entirely, which is the whole reason the
// arch form exists. Our FEM model has a single strength value (it doesn't
// distinguish tension from compression, a standard simplification for this
// class of algorithm), so using the TENSILE value here is the honest,
// conservative choice — the one that doesn't overstate what stone can
// actually take.
export const STONE_DENSITY_KG_M3 = 2600;
export const STONE_TENSILE_STRENGTH_PA = 5e6; // 5 MPa

export const BRIDGE_VOLUME_M3 = BRIDGE_SPAN_M * BRIDGE_RISE_M * BRIDGE_DEPTH_M;

// What the bridge would weigh at 100% material (nothing hollowed out).
export const MAX_MATERIAL_KG = BRIDGE_VOLUME_M3 * STONE_DENSITY_KG_M3;

export function kgToVolumeFraction(kg) {
  return kg / MAX_MATERIAL_KG;
}

export function volumeFractionToKg(fraction) {
  return fraction * MAX_MATERIAL_KG;
}

export const GRAVITY_M_S2 = 9.8;

// Real force (newtons) a real mass exerts under gravity.
export function kgToNewtons(kg) {
  return kg * GRAVITY_M_S2;
}

// The real force (newtons) a member can carry before its weakest point
// fails: tensile strength x cross-sectional area, where the area is that
// point's real width (a grid cell's real size, in meters) x the bridge's
// real depth x how filled that cell actually is (minDensity — SIMP's own
// density already represents a fraction of solid material, so a half-dense
// cell is treated as carrying half the force a fully solid one would).
//
// NOTE: this treats "one weakest cell" as the whole cross-section, which
// undersells a strut that got WIDER from more material budget — SIMP
// converges to near-binary (0 or 1) density almost everywhere, so more
// material mostly means more solid cells side-by-side, not a stronger
// single cell. Kept for cases (tests, simple examples) that only have a
// single-cell-wide member to reason about; maxForceFromArea below is the
// one that actually responds to material budget for a real strut.
export function maxForceNewtons(minDensity, widthM, depthM = BRIDGE_DEPTH_M) {
  const area = widthM * depthM * minDensity;
  return STONE_TENSILE_STRENGTH_PA * area;
}

// Capacity from a strut's total material instead of one weak point:
// average cross-section = total (density-weighted) area / real length.
// This is what actually responds to material budget — a strut built
// wider (more parallel solid cells, from a bigger volume fraction) gets a
// bigger average width and so a higher capacity, which a single cell's
// density can't reflect since SIMP pushes density to near-0-or-1 almost
// everywhere regardless of how much total material it had to work with.
export function maxForceFromArea(totalDensitySum, cellWidthM, gridLength, depthM = BRIDGE_DEPTH_M) {
  const avgWidthM = (totalDensitySum * cellWidthM) / gridLength;
  const area = avgWidthM * depthM;
  return STONE_TENSILE_STRENGTH_PA * area;
}
