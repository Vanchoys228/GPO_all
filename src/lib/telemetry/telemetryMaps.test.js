import { describe, expect, it } from "vitest";
import { INITIAL_TELEMETRY } from "./dashboardTelemetryState";
import { normalizeTelemetryMap } from "./telemetryMaps";

describe("telemetry maps", () => {
  it("normalizes cells and preserves previous metadata", () => {
    const result = normalizeTelemetryMap({
      totalCells: 1,
      cells: [{ x: "1", y: 2, confidence: -3 }, { x: "bad", y: 4 }],
    }, INITIAL_TELEMETRY.cameraMap);

    expect(result).toMatchObject({
      cellSize: 0.1,
      cellCount: 1,
      cells: [{ x: 1, y: 2, confidence: 0 }],
      imageFile: "camera_map.png",
    });
  });
});
