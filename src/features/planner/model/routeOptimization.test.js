import { describe, expect, it } from "vitest";
import { buildRouteOptimizationStatus } from "./routeOptimization";

describe("route optimization model", () => {
  const routed = {
    stationStopCount: 2,
    routeEnergy: 12.34,
    estimatedTimeSec: 45.67,
  };

  it("reports blocked routes before success details", () => {
    expect(
      buildRouteOptimizationStatus({ blocked: true, routed })
    ).toBe("Маршрут построен, но всё ещё пересекает ограничивающий контур.");
  });

  it("preserves route, charging, energy and adjustment details", () => {
    expect(
      buildRouteOptimizationStatus({
        adjustedVisitCount: 3,
        algorithmLabel: "GA + Tabu",
        routed,
        taskLabel: "Замкнутый маршрут",
      })
    ).toBe(
      "Маршрут построен: Замкнутый маршрут (GA + Tabu). 3 точек автоматически сдвинуты к безопасной позиции. Добавлено заездов на зарядку: 2. Энергия: 12.3 ед., время: 45.7 с."
    );
  });
});
