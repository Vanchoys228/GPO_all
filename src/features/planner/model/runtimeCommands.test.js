import { describe, expect, it } from "vitest";
import { buildMappingSurveyPayload, getMappingSurveyModeLabel } from "./runtimeCommands";

describe("runtime commands", () => {
  it("builds the mapping survey controller contract", () => {
    expect(
      buildMappingSurveyPayload({
        batteryRangeMeters: 150,
        commandId: 42,
        field: { minX: -1, maxX: 1, minY: -2, maxY: 2 },
        mode: "double",
        payloadKg: 5,
      })
    ).toEqual({
      type: "start_mapping_survey",
      commandId: 42,
      clearMap: true,
      mode: "double",
      field: { minX: -1, maxX: 1, minY: -2, maxY: 2 },
      motion: { cruiseSpeedMps: 0.8, payloadKg: 5, batteryRange: 150 },
    });
  });

  it("falls back to the first survey mode label", () => {
    expect(getMappingSurveyModeLabel("missing")).toBe("Змейка");
  });
});
