import { describe, expect, it } from "vitest";
import {
  buildSurfaceZonePayload,
  createSurfaceZoneDraft,
  deriveNextSurfaceZoneNumber,
  normalizeSurfaceZonesForImport,
  removeSurfaceZoneState,
  setSurfaceZoneClosed,
  setSurfaceZoneProfile,
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

  it("updates profile and closed state without mutating other zones", () => {
    const zones = [
      { id: "a", surfaceKey: "neutral", closed: false },
      { id: "b", surfaceKey: "rough", closed: true },
    ];
    expect(setSurfaceZoneProfile(zones, "a", "rough")[0].surfaceKey).toBe("rough");
    expect(setSurfaceZoneClosed(zones, "b", false)[1].closed).toBe(false);
    expect(zones[0].surfaceKey).toBe("neutral");
  });

  it("creates a fallback after removing the final surface zone", () => {
    expect(
      removeSurfaceZoneState({
        zones: [{ id: "surface-zone-3" }],
        zoneId: "surface-zone-3",
        activeZoneId: "surface-zone-3",
        fallbackSurfaceKey: "rough",
      })
    ).toEqual({
      zones: [createSurfaceZoneDraft(1, "rough")],
      activeZoneId: "surface-zone-1",
      activeSurfaceKey: "rough",
      nextZoneNumber: 2,
    });
  });
});
