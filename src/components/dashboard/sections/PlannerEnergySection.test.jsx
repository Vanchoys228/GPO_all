import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import PlannerEnergySection from "./PlannerEnergySection";

describe("PlannerEnergySection", () => {
  it("renders energy values and influence rows", () => {
    const html = renderToStaticMarkup(
      <PlannerEnergySection
        batteryRangeInput="120"
        cruiseSpeedInput="0.4"
        energyWarning=""
        onBatteryRangeBlur={vi.fn()}
        onBatteryRangeChange={vi.fn()}
        onCruiseSpeedBlur={vi.fn()}
        onCruiseSpeedChange={vi.fn()}
        onPayloadBlur={vi.fn()}
        onPayloadChange={vi.fn()}
        payloadInput="10"
        routeEnergyStats={{
          averageSlipRisk: 0.2,
          distanceMeters: 12,
          estimatedTimeSec: 90,
          limitingMaxSpeedMps: 0.4,
          routeEnergy: 8,
        }}
        routeInfluenceRows={[{ key: "surface", label: "Покрытие", value: "грунт", impact: "выше расход" }]}
        routeTiming={{ status: "finished", actualTimeSec: 95 }}
      />
    );
    expect(html).toContain("Энергия и динамика");
    expect(html).toContain("1 мин 30 сек");
    expect(html).toContain("Покрытие");
  });
});
