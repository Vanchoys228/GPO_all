import { describe, expect, it } from "vitest";
import normalization from "./number-normalization.cjs";

const { clamp, clampInt, normalizeNumber } = normalization;

describe("protocol number normalization", () => {
  it("clamps finite values to the requested range", () => {
    expect(clamp(-1, 0, 10)).toBe(0);
    expect(clamp(11, 0, 10)).toBe(10);
  });

  it("normalizes invalid values before integer clamping", () => {
    expect(normalizeNumber("1.5", 0)).toBe(1.5);
    expect(normalizeNumber("bad", 3)).toBe(3);
    expect(clampInt(Number.NaN, 4, 10)).toBe(4);
    expect(clampInt(8.6, 4, 10)).toBe(9);
  });
});
