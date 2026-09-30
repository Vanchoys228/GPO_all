import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";

import PlannerRoutePointsSection from "./PlannerRoutePointsSection";

describe("PlannerRoutePointsSection", () => {
  it("renders visit points before charge stations and keeps adjusted point details", () => {
    const html = renderToStaticMarkup(
      <PlannerRoutePointsSection
        chargeEntries={[{ index: 2, order: 1, point: { x: 5, y: 6 } }]}
        expandedPoint={0}
        hoveredPointIndex={0}
        onDeletePoint={vi.fn()}
        onHoverPoint={vi.fn()}
        onToggleExpandedPoint={vi.fn()}
        onUpdatePointTask={vi.fn()}
        plannedVisitEntries={[
          { index: 0, adjusted: true, plannedPoint: { x: 1.25, y: 2.5 } },
        ]}
        visitEntries={[
          { index: 0, order: 1, point: { x: 1, y: 2, task: "Ожидание 2 сек" } },
        ]}
      />,
    );

    expect(html.indexOf("Точки посещения")).toBeLessThan(html.indexOf("Станции зарядки"));
    expect(html).toContain("Автосдвиг");
    expect(html).toContain("Безопасная точка маршрута: x=1.2500, y=2.5000");
    expect(html).toContain("Ожидание 2 сек");
  });
});
