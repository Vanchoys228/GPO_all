import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import PlannerRouteControlsSection from "./PlannerRouteControlsSection";

describe("PlannerRouteControlsSection", () => {
  it("renders algorithm parameters and route actions in order", () => {
    const html = renderToStaticMarkup(
      <PlannerRouteControlsSection
        algorithmFields={[{ key: "iterations", label: "Итерации", min: 1, max: 10, step: 1 }]}
        hasRoute
        isOptimizing={false}
        onAddRandomObstacle={vi.fn()}
        onAlgorithmParamChange={vi.fn()}
        onClearAll={vi.fn()}
        onOptimizeRoute={vi.fn()}
        onSendRoute={vi.fn()}
        routeLength={12.5}
        selectedAlgorithmParams={{ iterations: 5 }}
      />
    );
    expect(html.indexOf("Параметры алгоритма")).toBeLessThan(html.indexOf("Построить маршрут"));
    expect(html).toContain("Итерации");
    expect(html).toContain("12.50 м");
  });
});
