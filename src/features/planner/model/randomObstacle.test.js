import { describe, expect, it } from "vitest";
import { pickRandomObstacleCenter, randomBetween } from "./randomObstacle";

describe("random obstacle placement", () => {
  it("supports deterministic random values", () => {
    expect(randomBetween(10, 20, () => 0.25)).toBe(12.5);
  });

  it("does not place an obstacle on top of the robot", () => {
    const result = pickRandomObstacleCenter({
      telemetry: { x: 0, y: 0 },
      optimizedRoute: [],
      points: [],
      polygons: [],
      obstacle: { sizeX: 0.5, sizeY: 0.5 },
      random: () => 0.5,
    });

    expect(result).toBeNull();
  });

  it("returns a valid free point", () => {
    const result = pickRandomObstacleCenter({
      telemetry: { x: -20, y: -15 },
      optimizedRoute: [],
      points: [],
      polygons: [],
      obstacle: { sizeX: 0.5, sizeY: 0.5 },
      random: () => 0.75,
    });

    expect(result).not.toBeNull();
    expect(Math.hypot(result.x + 20, result.y + 15)).toBeGreaterThan(1.1);
  });
});
