import { describe, expect, it } from "vitest";
import { normalizeAngle, pickNumber, toFiniteNumber } from "./telemetryNumbers";

describe("telemetry numbers", () => {
  it("normalizes finite values and selects the first valid candidate", () => {
    expect(toFiniteNumber(null)).toBeNull();
    expect(toFiniteNumber("2.5")).toBe(2.5);
    expect(pickNumber("bad", undefined, "3.5", 8)).toBe(3.5);
  });

  it("wraps angles into the signed pi range", () => {
    expect(normalizeAngle(Math.PI * 3)).toBe(Math.PI);
    expect(normalizeAngle(-Math.PI * 3)).toBe(-Math.PI);
  });
});
