import { describe, expect, it } from "vitest";
import {
  clampBatteryRange,
  mergeRoute,
  normalizeMandatoryRoute,
  normalizeStations,
} from "./chargingPlannerGeometry";

describe("charging planner geometry", () => {
  it("normalizes route points and removes adjacent duplicates", () => {
    expect(
      normalizeMandatoryRoute([
        { x: "0", y: "0" },
        { x: 0, y: 0 },
        { x: 2, y: 1 },
        { x: "invalid", y: 1 },
      ])
    ).toEqual([{ x: 0, y: 0 }, { x: 2, y: 1 }]);
  });

  it("deduplicates stations and enforces a positive battery range", () => {
    expect(normalizeStations([{ x: 1, y: 2 }, { x: "1", y: "2" }, { x: "x", y: 0 }]))
      .toEqual([{ x: 1, y: 2 }]);
    expect(clampBatteryRange(0)).toBeNull();
    expect(clampBatteryRange("2.5")).toBe(2.5);
  });

  it("merges connected route segments without duplicating the join point", () => {
    expect(mergeRoute([{ x: 0, y: 0 }, { x: 1, y: 0 }], [{ x: 1, y: 0 }, { x: 2, y: 0 }]))
      .toEqual([{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 }]);
  });
});
