import { describe, expect, it } from "vitest";
import routeValidation from "./route-validation.cjs";

const { validatePoints, validatePolygons, validateSurfaceZones } = routeValidation;

describe("route protocol validation", () => {
  it("normalizes valid points and rejects invalid or oversized routes", () => {
    expect(validatePoints([{ x: "1.5", y: 2 }])).toEqual([{ x: 1.5, y: 2 }]);
    expect(() => validatePoints([{ x: "bad", y: 2 }])).toThrow(/finite x and y/);
    expect(() => validatePoints(Array.from(
      { length: 1001 },
      (_, index) => ({ x: index, y: 0 })
    ))).toThrow(/at most 1000 points/);
  });

  it("validates limit and surface polygons", () => {
    expect(() => validatePolygons([{ points: [{ x: 0, y: 0 }] }])).toThrow(
      /at least three points/
    );
    const [zone] = validateSurfaceZones([{
      surfaceKey: "invalid",
      points: [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 0, y: 1 }],
    }]);
    expect(zone).toMatchObject({ id: "surface-zone-1", surfaceKey: "neutral" });
  });
});
