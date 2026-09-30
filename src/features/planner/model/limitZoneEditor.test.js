import { describe, expect, it } from "vitest";
import {
  createLimitZoneDraft,
  removeLimitZoneState,
  setLimitZoneClosed,
} from "./limitZoneEditor";

describe("limit zone editor model", () => {
  it("creates a stable limit-zone draft", () => {
    expect(createLimitZoneDraft(4)).toEqual({
      id: "zone-4",
      name: "Зона 4",
      closed: false,
    });
  });

  it("changes only the selected zone closed state", () => {
    const zones = [
      { id: "zone-1", closed: false },
      { id: "zone-2", closed: true },
    ];
    expect(setLimitZoneClosed(zones, "zone-2", false)).toEqual([
      zones[0],
      { id: "zone-2", closed: false },
    ]);
  });

  it("removes a zone, its points, and selects the remaining zone", () => {
    expect(
      removeLimitZoneState({
        zones: [{ id: "zone-1" }, { id: "zone-2" }],
        points: [
          { kind: "limit", zoneId: "zone-1" },
          { kind: "limit", zoneId: "zone-2" },
          { kind: "visit" },
        ],
        zoneId: "zone-1",
        activeZoneId: "zone-1",
      })
    ).toEqual({
      zones: [{ id: "zone-2" }],
      points: [{ kind: "limit", zoneId: "zone-2" }, { kind: "visit" }],
      activeZoneId: "zone-2",
    });
  });
});
