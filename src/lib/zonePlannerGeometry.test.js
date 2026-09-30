import { describe, expect, it } from "vitest";
import {
  pointInPolygon,
  segmentsIntersect,
} from "./zonePlannerGeometry";

describe("zone planner geometry", () => {
  it("treats polygon edges as inside", () => {
    const square = [
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      { x: 2, y: 2 },
      { x: 0, y: 2 },
    ];
    expect(pointInPolygon({ x: 1, y: 0 }, square)).toBe(true);
    expect(pointInPolygon({ x: 3, y: 1 }, square)).toBe(false);
  });

  it("detects touching and crossing segments", () => {
    expect(
      segmentsIntersect(
        { x: 0, y: 0 },
        { x: 2, y: 2 },
        { x: 0, y: 2 },
        { x: 2, y: 0 }
      )
    ).toBe(true);
  });
});
