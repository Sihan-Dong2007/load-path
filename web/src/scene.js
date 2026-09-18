// The fixed logical layout every layer (backdrop, structure, physics, water
// overlay) is drawn in. The page scales this 16:9 scene uniformly to fit
// the screen, so nothing ever stretches. All values are scene pixels.
export const SCENE_W = 1920;
export const SCENE_H = 1080;

// Where the FEM design domain sits: 2:1, matching the 2m x 1m bridge, with
// headroom above it so a weight resting on the apex stays on screen.
export const DOMAIN = { x: 240, y: 170, w: 1440, h: 720 };

// The bridge's two supports rest on cliffs whose tops are level with the
// bottom of the domain; between them is a river debris can fall into.
export const BANK_TOP_Y = DOMAIN.y + DOMAIN.h;
export const RIVER_LEFT_X = 300;
export const RIVER_RIGHT_X = 1620;
export const WATER_SURFACE_Y = 960;
export const RIVERBED_Y = 1040;
