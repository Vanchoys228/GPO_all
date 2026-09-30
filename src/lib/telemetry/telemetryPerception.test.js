import { describe, expect, it } from "vitest";
import { normalizeCamera, normalizeObstacleTrace } from "./telemetryPerception";

describe("telemetry perception", () => {
  it("retains the latest camera frame for metadata-only updates", () => {
    const previous = normalizeCamera({
      enabled: true,
      frameDataUrl: "data:image/jpeg;base64,abc",
      frameSequence: 1,
    });
    expect(normalizeCamera({ enabled: true, frameSequence: 2 }, previous)).toMatchObject({
      frameDataUrl: "data:image/jpeg;base64,abc",
      frameSequence: 2,
    });
  });

  it("filters invalid trace points and clamps confidence", () => {
    expect(normalizeObstacleTrace([
      { x: 1, y: 2, confidence: 4 },
      { x: "bad", y: 3 },
    ])).toEqual([{ x: 1, y: 2, confidence: 1 }]);
  });
});
