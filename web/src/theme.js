// Shared color palette for the "Chinese classical stone arch bridge" theme.
// Both the growth-phase canvas rendering (render.js) and the Matter.js
// collapse scene (collapse.js) read from here, so the two phases hand off
// to each other without a jarring color shift.
export const SKY_COLOR = "#0b1320";

export const STONE_COLOR = "#cfc3ab";
export const STONE_STROKE = "#8f8368";

// What a beam switches to the moment it's set dynamic — a visual cue, on
// top of the motion itself, that this piece is the one that failed.
export const BROKEN_STONE_COLOR = "#7d7060";
export const BROKEN_STONE_STROKE = "#4f463a";

export const GROUND_COLOR = "#241c14";
export const GROUND_STROKE = "#4a3c2a";

export const WEIGHT_COLOR = "#6b5a3a";
export const WEIGHT_STROKE = "#392f1c";

// The puff of debris a shard kicks up the instant it breaks.
export const DUST_COLOR = "#b7ab94";
