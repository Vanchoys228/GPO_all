import { describe, expect, it } from "vitest";
import * as zonePlanner from "./zonePlanner";
import { sanitizeRouteForController } from "./zonePlanner";

describe("zonePlanner public facade", () => {
  it("preserves the public export contract", () => {
    expect(Object.keys(zonePlanner).sort()).toEqual([
      "CANVAS_HEIGHT", "CANVAS_WIDTH", "CONTROLLER_MIN_SEGMENT",
      "DEFAULT_POINT_TASK", "DEFAULT_SURFACE_ZONES", "DRAWING_HEIGHT",
      "DRAWING_LEFT", "DRAWING_TOP", "DRAWING_WIDTH", "HALF_HEIGHT",
      "HALF_WIDTH", "MAP_PADDING", "MAP_WORLD_HEIGHT", "MAP_WORLD_WIDTH",
      "POINT_KIND_META", "POINT_KIND_OPTIONS", "POINT_TASKS",
      "ROUTE_CLEARANCE_MARGIN", "SAFE_POINT_MARGIN", "SCALE",
      "buildObstacleAwareRoute", "canvasToWorld", "dist", "drawDiamond",
      "drawPlannerBackground", "getZoneColor", "isInsideMap", "pointEquals",
      "pointInAnyPolygon", "pointInPolygon", "projectPointOutsidePolygons",
      "routeCrossesAnyLimitPolygon", "sanitizeRouteForController",
      "segmentsIntersect", "worldToCanvas",
    ]);
  });
});

describe("sanitizeRouteForController", () => {
  it("preserves the final return to the start for tsp routes", () => {
    const route = [
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      { x: 2, y: 2 },
      { x: 0, y: 0 },
    ];

    expect(sanitizeRouteForController(route)).toEqual(route);
  });
});
