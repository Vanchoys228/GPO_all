import { describe, expect, it } from "vitest";
import { ALGORITHM_OPTIONS } from "../../../lib/routeAlgorithms";
import { createDashboardAlgorithmParams } from "./dashboardAlgorithmParams";

describe("dashboard algorithm parameters", () => {
  it("creates an independent default parameter object for every algorithm", () => {
    const params = createDashboardAlgorithmParams();
    expect(Object.keys(params)).toEqual(ALGORITHM_OPTIONS.map((option) => option.key));
    expect(params[ALGORITHM_OPTIONS[0].key]).not.toBe(params[ALGORITHM_OPTIONS[1].key]);
  });
});
