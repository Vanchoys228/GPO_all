export const MAP_WORLD_WIDTH = 44;
export const MAP_WORLD_HEIGHT = 34;
export const HALF_WIDTH = MAP_WORLD_WIDTH / 2;
export const HALF_HEIGHT = MAP_WORLD_HEIGHT / 2;
export const isInsideMap = point => point.x >= -HALF_WIDTH && point.x <= HALF_WIDTH && point.y >= -HALF_HEIGHT && point.y <= HALF_HEIGHT;
