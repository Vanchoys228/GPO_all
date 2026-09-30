import { describe, expect, it } from "vitest";
import mappingValidation from "./mapping-validation.cjs";

const { sanitizeMappingSurveyMode } = mappingValidation;

describe("mapping protocol validation", () => {
  it("accepts supported modes and defaults unknown values", () => {
    expect(sanitizeMappingSurveyMode(" DOUBLE ")).toBe("double");
    expect(sanitizeMappingSurveyMode("unknown")).toBe("snake");
    expect(sanitizeMappingSurveyMode()).toBe("snake");
  });
});
