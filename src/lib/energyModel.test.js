import { describe, expect, it } from "vitest";
import {
  SURFACE_ZONE_PRESETS,
  estimateRouteEnergy,
  resolveSurfaceAtPoint,
} from "./energyModel";

describe("energyModel", () => {
  it("resolves surface profile by point on map", () => {
    const roughPoint = { x: -10, y: -10 };
    const neutralPoint = { x: 10, y: -10 };

    const rough = resolveSurfaceAtPoint(roughPoint, SURFACE_ZONE_PRESETS);
    const neutral = resolveSurfaceAtPoint(neutralPoint, SURFACE_ZONE_PRESETS);

    expect(rough.profile.key).toBe("rough");
    expect(neutral.profile.key).toBe("neutral");
  });

  it("increases energy when payload and speed rise", () => {
    const route = [
      { x: 0, y: 0 },
      { x: 8, y: 0 },
      { x: 8, y: 8 },
    ];

    const light = estimateRouteEnergy(route, {
      surfaceZones: [],
      speedMps: 0.18,
      payloadKg: 0,
    });
    const heavyFast = estimateRouteEnergy(route, {
      surfaceZones: [],
      speedMps: 0.36,
      payloadKg: 30,
    });

    expect(heavyFast.totalEnergy).toBeGreaterThan(light.totalEnergy);
  });

  it("accounts for rough surface on the route", () => {
    const routeInsideRough = [
      { x: -18, y: -12 },
      { x: -8, y: -12 },
    ];
    const routeNeutral = [
      { x: 10, y: -12 },
      { x: 20, y: -12 },
    ];

    const rough = estimateRouteEnergy(routeInsideRough, {
      surfaceZones: SURFACE_ZONE_PRESETS,
      speedMps: 0.18,
      payloadKg: 0,
    });
    const neutral = estimateRouteEnergy(routeNeutral, {
      surfaceZones: SURFACE_ZONE_PRESETS,
      speedMps: 0.18,
      payloadKg: 0,
    });

    expect(rough.totalEnergy).toBeGreaterThan(neutral.totalEnergy);
    expect(rough.limitingMaxSpeedMps).toBeLessThanOrEqual(neutral.limitingMaxSpeedMps);
  });
});
