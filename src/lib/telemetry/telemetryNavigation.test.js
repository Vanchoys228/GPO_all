import { describe, expect, it } from "vitest";
import { normalizeNavigation } from "./telemetryNavigation";

describe("telemetry navigation", () => {
  it("normalizes provided state and retains optional previous metrics", () => {
    expect(normalizeNavigation(
      { status: "tracking", finished: false, currentWaypointIndex: "2" },
      { distanceToTarget: 3.5 }
    )).toEqual({
      status: "tracking",
      finished: false,
      currentWaypointIndex: 2,
      distanceToTarget: 3.5,
    });
  });
});
