import { MAP_WORLD_WIDTH, MAP_WORLD_HEIGHT, HALF_WIDTH, HALF_HEIGHT } from "../../shared/planning/worldBounds.js";
export { MAP_WORLD_WIDTH, MAP_WORLD_HEIGHT, HALF_WIDTH, HALF_HEIGHT, isInsideMap } from "../../shared/planning/worldBounds.js";
export const CANVAS_WIDTH = 1180;
export const CANVAS_HEIGHT = 920;
export const MAP_PADDING = 44;
export const SCALE = Math.min(
  (CANVAS_WIDTH - MAP_PADDING * 2) / MAP_WORLD_WIDTH,
  (CANVAS_HEIGHT - MAP_PADDING * 2) / MAP_WORLD_HEIGHT
);
export const DRAWING_WIDTH = MAP_WORLD_WIDTH * SCALE;
export const DRAWING_HEIGHT = MAP_WORLD_HEIGHT * SCALE;
export const DRAWING_LEFT = (CANVAS_WIDTH - DRAWING_WIDTH) / 2;
export const DRAWING_TOP = (CANVAS_HEIGHT - DRAWING_HEIGHT) / 2;

export const worldToCanvas = (x, y) => ({
  x: DRAWING_LEFT + (x + HALF_WIDTH) * SCALE,
  y: DRAWING_TOP + (HALF_HEIGHT - y) * SCALE,
});

export const canvasToWorld = (x, y) => ({
  x: (x - DRAWING_LEFT) / SCALE - HALF_WIDTH,
  y: HALF_HEIGHT - (y - DRAWING_TOP) / SCALE,
});
