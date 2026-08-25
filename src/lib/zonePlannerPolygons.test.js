import { describe, expect, it } from "vitest";
import { pointInPolygon } from "./zonePlannerGeometry";
import {
  projectPointOutsidePolygons,
  routeCrossesAnyLimitPolygon,
} from "./zonePlannerPolygons";

const squareZone = {
  id: "zone-1",
  points: [
    { x: -1, y: -1 },
    { x: 1, y: -1 },
    { x: 1, y: 1 },
    { x: -1, y: 1 },
  ],
};

describe("zone planner polygons", () => {
  it("projects blocked points outside the polygon margin", () => {
    const result = projectPointOutsidePolygons({ x: 0, y: 0 }, [squareZone], 0.5);
    expect(result.adjusted).toBe(true);
    expect(pointInPolygon(result.point, squareZone.points)).toBe(false);
  });

  it("detects a route crossing a limiting polygon", () => {
    expect(
      routeCrossesAnyLimitPolygon(
        [
          { x: -3, y: 0 },
          { x: 3, y: 0 },
        ],
        [squareZone]
      )
    ).toBe(true);
    expect(
      routeCrossesAnyLimitPolygon(
        [
          { x: -3, y: 3 },
          { x: 3, y: 3 },
        ],
        [squareZone]
      )
    ).toBe(false);
  });
});
