import { describe, expect, it } from "vitest";
import { describeSurfaceRuntime, estimateRouteEnergy } from "./energyEstimator";

describe("energy estimator", () => {
  it("caps effective speed by the active surface profile", () => {
    const runtime = describeSurfaceRuntime("slippery", { speedMps: 0.6, payloadKg: 5 });
    expect(runtime.effectiveSpeedMps).toBe(0.16);
    expect(runtime.energyMultiplier).toBeGreaterThan(1);
  });

  it("adds turn time only when the turn penalty is enabled", () => {
    const route = [{ x: 0, y: 0 }, { x: 3, y: 0 }, { x: 3, y: 3 }];
    const withTurns = estimateRouteEnergy(route, { surfaceZones: [], speedMps: 0.2 });
    const withoutTurns = estimateRouteEnergy(route, { surfaceZones: [], speedMps: 0.2, includeTurnPenalty: false });
    expect(withTurns.totalEnergy).toBeGreaterThan(withoutTurns.totalEnergy);
    expect(withTurns.estimatedTimeSec).toBeGreaterThan(withoutTurns.estimatedTimeSec);
  });
});
