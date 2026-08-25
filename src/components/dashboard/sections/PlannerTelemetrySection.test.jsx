import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";

import PlannerTelemetrySection from "./PlannerTelemetrySection";

describe("PlannerTelemetrySection", () => {
  it("renders telemetry, camera, map controls and connection states", () => {
    const html = renderToStaticMarkup(
      <PlannerTelemetrySection
        mapExportPromptOpen
        mappingSurveyMode="double"
        mappingSurveyModes={[{ key: "double", label: "Двойной" }]}
        onCancelMapExport={vi.fn()}
        onExportMapVariant={vi.fn()}
        onMappingSurveyModeChange={vi.fn()}
        onRequestMapExport={vi.fn()}
        onStartMappingSurvey={vi.fn()}
        routeWsUp={false}
        solverApiUp
        telemetry={{
          x: 1,
          y: 2,
          z: 3,
          yaw: 4,
          perception: {
            lidar: { enabled: true },
            camera: {
              enabled: true,
              mode: "virtual_lidar",
              obstacleVisible: true,
              obstacleRange: 1.25,
              obstacleScore: 0.8,
              detectionCount: 2,
              frameDataUrl: "data:image/png;base64,frame",
            },
          },
          obstacleTrace: [{ x: 0, y: 0 }],
          obstacleMap: { cells: [{ x: 0, y: 0 }], cellSize: 0.1 },
          cameraMap: { cells: [{ x: 0, y: 0 }], freeCells: [{ x: 1, y: 1 }] },
          navigation: { avoidanceTimeSec: 12.4 },
        }}
        telemetryWsUp
      />,
    );

    expect(html).toContain("Телеметрия");
    expect(html).toContain("camera: virtual");
    expect(html).toContain("cam dist: 1.25 m");
    expect(html).toContain("Время объезда вне маршрута:");
    expect(html).toContain("Кадр с камеры робота");
    expect(html).toContain("Двойной объезд");
    expect(html).toContain("Какую карту сохранить?");
    expect(html).toContain("WS Telemetry:");
    expect(html).toContain("Native Solver:");
  });
});
