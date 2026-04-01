import { describe, expect, it } from "vitest";
import {
  buildPlannerModel,
  getRouteAnchor,
  INITIAL_ZONE,
  rotateClosedRouteToNearestPoint,
} from "./plannerModel";

describe("buildPlannerModel", () => {
  it("builds a consistent planner model for zones, visits, charging and route checks", () => {
    const limitZones = [{ ...INITIAL_ZONE, closed: true }];
    const points = [
      { kind: "limit", zoneId: INITIAL_ZONE.id, x: -1, y: -1 },
      { kind: "limit", zoneId: INITIAL_ZONE.id, x: 1, y: -1 },
      { kind: "limit", zoneId: INITIAL_ZONE.id, x: 1, y: 1 },
      { kind: "limit", zoneId: INITIAL_ZONE.id, x: -1, y: 1 },
      { kind: "visit", x: 0, y: 0, task: "wait" },
      { kind: "visit", x: 3, y: 0, task: "scan" },
      { kind: "charge", x: -3, y: -2 },
    ];
    const optimizedRoute = [
      { x: -2, y: 0 },
      { x: 2, y: 0 },
      { x: 3, y: 0 },
    ];

    const model = buildPlannerModel({
      points,
      limitZones,
      optimizedRoute,
      activeLimitZoneId: INITIAL_ZONE.id,
    });

    expect(model.zoneEntries).toHaveLength(1);
    expect(model.polygons).toHaveLength(1);
    expect(model.visitEntries).toHaveLength(2);
    expect(model.chargeEntries).toHaveLength(1);
    expect(model.chargePoints).toEqual([{ x: -3, y: -2 }]);
    expect(model.visitsInsideLimit).toHaveLength(1);
    expect(model.adjustedVisits).toHaveLength(1);
    expect(model.routeBlocked).toBe(true);
    expect(model.activeZoneName).toBe("Зона 1");
    expect(model.plannedVisitEntries[0].plannedPoint).not.toEqual(model.visitEntries[0].point);
  });

  it("creates a live preview obstacle from the first open-zone point", () => {
    const limitZones = [{ ...INITIAL_ZONE, closed: false }];
    const points = [
      { kind: "limit", zoneId: INITIAL_ZONE.id, x: 0, y: 0 },
      { kind: "visit", x: 0.1, y: 0.1, task: "wait" },
      { kind: "visit", x: 3, y: 0, task: "scan" },
    ];

    const model = buildPlannerModel({
      points,
      limitZones,
      optimizedRoute: [],
      activeLimitZoneId: INITIAL_ZONE.id,
    });

    expect(model.polygons).toHaveLength(0);
    expect(model.previewPolygons).toHaveLength(1);
    expect(model.previewPolygons[0].preview).toBe(true);
    expect(model.previewPolygons[0].points).toHaveLength(4);
    expect(model.adjustedVisits).toHaveLength(1);
  });
});

describe("rotateClosedRouteToNearestPoint", () => {
  it("rotates the loop so it starts from the closest point to the robot", () => {
    const route = [
      { x: 5, y: 0 },
      { x: 1, y: 0 },
      { x: 3, y: 0 },
    ];

    const rotated = rotateClosedRouteToNearestPoint(route, { x: 0, y: 0 });

    expect(rotated).toEqual([
      { x: 1, y: 0 },
      { x: 3, y: 0 },
      { x: 5, y: 0 },
    ]);
  });

  it("preserves loop closure for tsp routes", () => {
    const route = [
      { x: 5, y: 0 },
      { x: 1, y: 0 },
      { x: 3, y: 0 },
      { x: 5, y: 0 },
    ];

    const rotated = rotateClosedRouteToNearestPoint(route, { x: 0, y: 0 });

    expect(rotated).toEqual([
      { x: 1, y: 0 },
      { x: 3, y: 0 },
      { x: 5, y: 0 },
      { x: 1, y: 0 },
    ]);
  });
});

describe("getRouteAnchor", () => {
  it("falls back to the origin when telemetry is incomplete", () => {
    expect(getRouteAnchor({ x: 3, y: 4 })).toEqual({ x: 3, y: 4 });
    expect(getRouteAnchor({ x: "bad", y: 5 })).toEqual({ x: 0, y: 5 });
    expect(getRouteAnchor(null)).toEqual({ x: 0, y: 0 });
  });
});
