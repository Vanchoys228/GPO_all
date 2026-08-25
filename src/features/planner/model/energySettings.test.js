import { describe, expect, it } from "vitest";
import {
  normalizeBatteryRange,
  normalizeCruiseSpeed,
  normalizePayload,
} from "./energySettings";

describe("energy settings", () => {
  it("accepts comma decimals and clamps controller ranges", () => {
    expect(normalizeBatteryRange("0,4")).toBe(1);
    expect(normalizeCruiseSpeed("1,2")).toBe(0.8);
    expect(normalizePayload("501")).toBe(500);
  });

  it("keeps invalid input distinguishable", () => {
    expect(normalizeCruiseSpeed("")).toBeNull();
  });
});
