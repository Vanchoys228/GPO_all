import { describe, expect, it } from "vitest";
import { findLegFrontier } from "./chargingPlannerLegs";

describe("charging planner legs", () => {
  it("builds a reachable direct leg as a frontier option", () => {
    const options = findLegFrontier({
      start: { x: 0, y: 0 },
      end: { x: 5, y: 0 },
      startFuel: 100,
      stations: [],
      polygons: [],
      batteryRange: 100,
      surfaceZones: [],
      energyOptions: {},
      segmentCache: new Map(),
    });

    expect(options).toHaveLength(1);
    expect(options[0].route).toEqual([{ x: 0, y: 0 }, { x: 5, y: 0 }]);
    expect(options[0].stationStops).toBe(0);
  });
});
