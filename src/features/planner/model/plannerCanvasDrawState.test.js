import { describe, expect, it } from "vitest";
import { createPlannerCanvasDrawState } from "./plannerCanvasDrawState";

describe("planner canvas draw state", () => {
  it("combines planner and telemetry data into a render-safe state", () => {
    const plannerModel = {
      visitEntries: [{ index: 1 }],
      chargeEntries: [{ index: 2 }],
      plannedVisitEntryMap: new Map([[1, { adjusted: true }]]),
      zoneEntries: [{ name: "Restricted" }],
      surfaceZones: [{ name: "Concrete" }],
      routeBlocked: true,
    };

    expect(
      createPlannerCanvasDrawState({
        plannerModel,
        optimizedRoute: [{ x: 1, y: 2 }],
        hoveredPointIndex: 1,
        telemetry: {},
        fallbackObstacleMap: { cells: [{ x: 0, y: 0 }] },
        fallbackCameraMap: { cells: [{ x: 3, y: 4 }] },
      })
    ).toEqual({
      visitEntries: plannerModel.visitEntries,
      chargeEntries: plannerModel.chargeEntries,
      plannedVisitEntryMap: plannerModel.plannedVisitEntryMap,
      zoneEntries: plannerModel.zoneEntries,
      surfaceZones: plannerModel.surfaceZones,
      optimizedRoute: [{ x: 1, y: 2 }],
      obstacleTrace: [],
      obstacleMap: { cells: [{ x: 0, y: 0 }] },
      cameraMap: { cells: [{ x: 3, y: 4 }] },
      routeBlocked: true,
      hoveredPointIndex: 1,
    });
  });
});
