import { describe, expect, it } from "vitest";
import {
  DEFAULT_BATTERY_RANGE_METERS,
  planRouteWithCharging,
} from "./chargingPlanner";

describe("planRouteWithCharging", () => {
  it("keeps the route unchanged when battery range is sufficient", () => {
    const route = [
      { x: 0, y: 0 },
      { x: 10, y: 0 },
      { x: 18, y: 0 },
    ];

    const result = planRouteWithCharging({
      route,
      stations: [],
      polygons: [],
      batteryRange: 25,
    });

    expect(result.ok).toBe(true);
    expect(result.route).toEqual(route);
    expect(result.stationStopCount).toBe(0);
  });

  it("reports unreachable route when no charging stations are available", () => {
    const route = [
      { x: 0, y: 0 },
      { x: 70, y: 0 },
      { x: 120, y: 0 },
    ];

    const result = planRouteWithCharging({
      route,
      stations: [],
      polygons: [],
      batteryRange: 100,
    });

    expect(result.ok).toBe(false);
    expect(result.reason).toBe("insufficient_range");
    expect(result.error).toMatch(/недостижим/i);
  });

  it("inserts charging stops when stations make the route feasible", () => {
    const route = [
      { x: 0, y: 0 },
      { x: 80, y: 0 },
      { x: 160, y: 0 },
    ];
    const stations = [{ x: 80, y: 10 }];

    const result = planRouteWithCharging({
      route,
      stations,
      polygons: [],
      batteryRange: DEFAULT_BATTERY_RANGE_METERS,
    });

    expect(result.ok).toBe(true);
    expect(result.stationStopCount).toBeGreaterThan(0);
    expect(result.route.some((point) => point.x === 80 && point.y === 10)).toBe(true);
    expect(result.route[0]).toEqual(route[0]);
  });

  it("returns clear error for invalid battery range", () => {
    const route = [
      { x: 0, y: 0 },
      { x: 10, y: 0 },
      { x: 20, y: 0 },
    ];

    const result = planRouteWithCharging({
      route,
      stations: [{ x: 5, y: 0 }],
      polygons: [],
      batteryRange: 0,
    });

    expect(result.ok).toBe(false);
    expect(result.reason).toBe("invalid_battery_range");
    expect(result.error).toMatch(/запас хода/i);
  });

  it("prefers charging station that is closer to the leg geometry", () => {
    const route = [
      { x: 0, y: 0 },
      { x: 140, y: 0 },
    ];
    const stations = [
      { x: 70, y: 4 },
      { x: 68, y: 20 },
    ];

    const result = planRouteWithCharging({
      route,
      stations,
      polygons: [],
      batteryRange: 95,
    });

    expect(result.ok).toBe(true);
    expect(result.route.some((point) => point.x === 70 && point.y === 4)).toBe(true);
    expect(result.route.some((point) => point.x === 68 && point.y === 20)).toBe(false);
  });
});
