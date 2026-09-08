import { describe, expect, it } from "vitest";
import { buildSceneSnapshot } from "./sceneSnapshot";

describe("scene snapshot", () => {
  it("excludes editor drafts while retaining closed surface geometry", () => {
    const points = [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 0, y: 1 }];
    const closed = { id: "floor", surfaceKey: "rough", points, closed: true };
    const scene = buildSceneSnapshot({
      plannerModel: { surfaceZones: [closed, { points: [] }, { points, closed: false }] },
      batteryRangeMeters: 100,
    });
    expect(scene.surfaceZones).toEqual([closed]);
  });
});
