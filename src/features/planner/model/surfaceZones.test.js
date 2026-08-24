import { describe, expect, it } from "vitest";
import {
  buildSurfaceZonePayload,
  createSurfaceZoneDraft,
  deriveNextSurfaceZoneNumber,
  normalizeSurfaceZonesForImport,
} from "./surfaceZones";

describe("surface zone model", () => {
  it("creates stable drafts and derives the next number", () => {
    expect(createSurfaceZoneDraft(4)).toMatchObject({
      id: "surface-zone-4",
      name: "Покрытие 4",
      closed: false,
    });
    expect(deriveNextSurfaceZoneNumber([{ id: "surface-zone-7" }])).toBe(8);
  });

  it("only sends closed polygons with at least three points", () => {
    const points = [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 0, y: 1 }];
    expect(
      buildSurfaceZonePayload([
        { id: "closed", name: "Closed", surfaceKey: "rough", closed: true, points },
        { id: "draft", name: "Draft", surfaceKey: "rough", closed: false, points },
      ]).zones
    ).toHaveLength(1);
  });

  it("normalizes imported zones and drops empty entries", () => {
    expect(
      normalizeSurfaceZonesForImport([
        { id: "rough", surfaceKey: "rough", points: [{ x: 0, y: 0 }] },
        { id: "empty", points: [] },
      ])
    ).toHaveLength(1);
  });
});
