import { describe, expect, it } from "vitest";
import { resolveSurfaceAtPoint } from "./energySurfaceZones";

describe("energy surface zones", () => {
  it("uses the latest closed zone that contains the sampled point", () => {
    const zones = [
      { surfaceKey: "rough", points: [{ x: 0, y: 0 }, { x: 4, y: 0 }, { x: 4, y: 4 }, { x: 0, y: 4 }] },
      { surfaceKey: "slippery", points: [{ x: 1, y: 1 }, { x: 3, y: 1 }, { x: 3, y: 3 }, { x: 1, y: 3 }] },
    ];

    expect(resolveSurfaceAtPoint({ x: 2, y: 2 }, zones).profile.key).toBe("slippery");
    expect(resolveSurfaceAtPoint({ x: 8, y: 8 }, zones).profile.key).toBe("neutral");
  });
});
