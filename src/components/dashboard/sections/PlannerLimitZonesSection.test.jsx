import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import PlannerLimitZonesSection from "./PlannerLimitZonesSection";

describe("PlannerLimitZonesSection", () => {
  it("renders active-zone summary before the zone list", () => {
    const zone = { id: "zone-1", name: "Зона 1", closed: true, points: [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 1, y: 1 }], color: { badge: "bg-blue-500" } };
    const html = renderToStaticMarkup(
      <PlannerLimitZonesSection
        activeLimitZoneId={zone.id}
        activeZone={zone}
        activeZoneName={zone.name}
        onClearZone={vi.fn()}
        onCreateZone={vi.fn()}
        onRemoveZone={vi.fn()}
        onSelectZone={vi.fn()}
        onToggleZoneClosed={vi.fn()}
        zoneEntries={[zone]}
      />
    );
    expect(html.indexOf("Активная зона")).toBeLessThan(html.indexOf("Ограничивающие зоны"));
    expect(html).toContain("Контур готов");
  });
});
