import { describe, expect, it } from "vitest";
import {
  DEFAULT_ENERGY_OPTIONS,
  SURFACE_PROFILE_OPTIONS,
  getSurfaceProfileByKey,
} from "./energyProfiles";

describe("energy profiles", () => {
  it("returns a neutral fallback profile for an unknown surface", () => {
    expect(getSurfaceProfileByKey("missing").key).toBe("neutral");
    expect(SURFACE_PROFILE_OPTIONS.map((profile) => profile.key)).toEqual([
      "neutral", "rough", "slippery",
    ]);
  });

  it("exposes default route energy inputs", () => {
    expect(DEFAULT_ENERGY_OPTIONS).toEqual({ speedMps: 0.22, payloadKg: 0 });
  });
});
