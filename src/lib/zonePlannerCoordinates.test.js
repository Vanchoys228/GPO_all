import { describe, expect, it } from "vitest";
import {
  HALF_HEIGHT,
  HALF_WIDTH,
  canvasToWorld,
  isInsideMap,
  worldToCanvas,
} from "./zonePlannerCoordinates";

describe("zone planner coordinates", () => {
  it("round-trips world coordinates through the canvas", () => {
    const world = { x: 7.25, y: -3.5 };
    const canvas = worldToCanvas(world.x, world.y);
    expect(canvasToWorld(canvas.x, canvas.y)).toEqual(world);
  });

  it("keeps the map boundary inclusive", () => {
    expect(isInsideMap({ x: HALF_WIDTH, y: HALF_HEIGHT })).toBe(true);
    expect(isInsideMap({ x: HALF_WIDTH + 0.001, y: 0 })).toBe(false);
  });
});
