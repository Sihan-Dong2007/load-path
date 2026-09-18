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
// Needed specifically for buckling — unlike the strength check, buckling
// resistance depends on how STIFF a member is (E), not just how strong it
// is, so this is a genuinely new physical constant, not a restatement of
// the tensile strength above. A representative value for granite/limestone.
export const STONE_ELASTIC_MODULUS_PA = 3e10; // 30 GPa

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
