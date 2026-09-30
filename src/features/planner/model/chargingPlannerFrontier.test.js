import { describe, expect, it } from "vitest";
import { keepFrontierBounded, pruneDominatedLabels } from "./chargingPlannerFrontier";

describe("charging planner frontier", () => {
  it("removes a label that costs more and leaves less fuel", () => {
    expect(pruneDominatedLabels([
      { cost: 10, fuel: 8, stationStops: 1 },
      { cost: 11, fuel: 7, stationStops: 0 },
      { cost: 9, fuel: 6, stationStops: 1 },
    ])).toEqual([
      { cost: 9, fuel: 6, stationStops: 1 },
      { cost: 10, fuel: 8, stationStops: 1 },
    ]);
  });

  it("bounds the retained frontier by cost and available fuel", () => {
    expect(keepFrontierBounded([
      { cost: 3, fuel: 1, stationStops: 0 },
      { cost: 1, fuel: 1, stationStops: 0 },
      { cost: 2, fuel: 2, stationStops: 0 },
    ], 2)).toHaveLength(2);
  });
});
