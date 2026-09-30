import { describe, expect, it } from "vitest";
import { analyzeRouteInfluence } from "./energyInfluence";

describe("energy influence", () => {
  it("reports actual time only when a positive actual measurement is provided", () => {
    const route = [{ x: 0, y: 0 }, { x: 2, y: 0 }];
    const withoutActual = analyzeRouteInfluence(route, { surfaceZones: [] });
    const withActual = analyzeRouteInfluence(route, { surfaceZones: [], plannedTimeSec: 8, actualTimeSec: 10 });

    expect(withoutActual.some((row) => row.key === "actual-time")).toBe(false);
    expect(withActual.find((row) => row.key === "actual-time")).toMatchObject({ value: "10 сек", impact: "+2 сек" });
  });
});
