import { describe, expect, it } from "vitest";
import { isNavigationOffRoute } from "./useRouteTiming";

describe("isNavigationOffRoute", () => {
  it("detects explicit avoidance flags", () => {
    expect(isNavigationOffRoute({ avoidanceActive: true })).toBe(true);
    expect(isNavigationOffRoute({ offRouteActive: true })).toBe(true);
  });

  it("detects controller avoidance statuses", () => {
    expect(isNavigationOffRoute({ status: "avoiding_obstacle" })).toBe(true);
    expect(isNavigationOffRoute({ status: "passing_lidar_gap" })).toBe(true);
    expect(isNavigationOffRoute({ status: "following_route" })).toBe(false);
  });
});
