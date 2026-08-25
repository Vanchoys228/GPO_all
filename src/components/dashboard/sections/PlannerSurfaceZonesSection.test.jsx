import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import PlannerSurfaceZonesSection from "./PlannerSurfaceZonesSection";

describe("PlannerSurfaceZonesSection", () => {
  it("renders surface zones and active runtime information", () => {
    const zone = { id: "surface-1", name: "Грунт", surfaceKey: "soil", closed: true, points: [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 1, y: 1 }] };
    const html = renderToStaticMarkup(
      <PlannerSurfaceZonesSection
        activeSurfaceProfileKey="soil"
        activeSurfaceZone={zone}
        activeSurfaceZoneId={zone.id}
        cruiseSpeedInput="0.4"
        cruiseSpeedMps={0.4}
        onActiveSurfaceProfileChange={vi.fn()}
        onClearAllSurfaceZones={vi.fn()}
        onClearSurfaceZone={vi.fn()}
        onCreateSurfaceZone={vi.fn()}
        onRemoveSurfaceZone={vi.fn()}
        onSelectSurfaceZone={vi.fn()}
        onToggleSurfaceZoneClosed={vi.fn()}
        payloadInput="10"
        payloadKg={10}
        surfaceZones={[zone]}
      />
    );
    expect(html).toContain("Покрытия карты");
    expect(html).toContain("Редактируемые зоны: 1");
    expect(html).toContain("Грунт");
  });
});
