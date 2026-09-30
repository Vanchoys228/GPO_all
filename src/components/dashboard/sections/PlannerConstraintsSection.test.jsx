import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";

import PlannerConstraintsSection from "./PlannerConstraintsSection";

describe("PlannerConstraintsSection", () => {
  it("renders route constraint counters and blocked state", () => {
    const html = renderToStaticMarkup(
      <PlannerConstraintsSection
        adjustedVisitCount={3}
        polygonCount={2}
        routeBlocked
        visitsInsideLimitCount={1}
      />,
    );

    expect(html).toContain("Контроль ограничений");
    expect(html).toContain("Точек внутри зон: <b>1</b>");
    expect(html).toContain("Контуров, готовых для обхода: <b>2</b>");
    expect(html).toContain("Точек с автосдвигом: <b>3</b>");
    expect(html).toContain("Маршрут пересекает контур: <b>да</b>");
  });
});
