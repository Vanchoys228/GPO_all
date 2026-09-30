import { describe, expect, it } from "vitest";
import { hasMapData, normalizeMapExportVariant } from "./mapExport";

describe("map export model", () => {
  it("recognizes obstacle and free camera cells", () => {
    expect(hasMapData({ cells: [] })).toBe(false);
    expect(hasMapData({ freeCells: [{ x: 1, y: 2 }] })).toBe(true);
  });

  it("allows only camera or lidar variants", () => {
    expect(normalizeMapExportVariant("camera")).toBe("camera");
    expect(normalizeMapExportVariant("unknown")).toBe("lidar");
  });
});
