import { describe, expect, it } from "vitest";
import { buildPlannerSyncPayloads } from "./plannerSync";

describe("planner sync payloads", () => {
  it("rounds controller coordinates and preserves payload types", () => {
    const payloads = buildPlannerSyncPayloads({
      chargePoints: [{ x: 1.23456, y: 2.34567 }],
      polygons: [
        { id: "z", name: "Z", points: [{ x: 3.45678, y: 4.56789 }] },
      ],
      previewPolygons: [],
      surfaceZones: [],
    });

    expect(JSON.parse(payloads.zoneSyncPayloadText)).toMatchObject({
      type: "limit_zones",
      zones: [{ points: [{ x: 3.4568, y: 4.5679 }] }],
    });
    expect(JSON.parse(payloads.chargePointsRoutingText)).toEqual([
      { x: 1.2346, y: 2.3457 },
    ]);
  });
});
