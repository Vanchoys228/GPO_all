import { describe, expect, it, vi } from "vitest";
import { startPlannerCanvasRenderLoop } from "./plannerCanvasRenderLoop";

const createContext = () => ({
  arc: vi.fn(),
  beginPath: vi.fn(),
  createLinearGradient: vi.fn(() => ({ addColorStop: vi.fn() })),
  fill: vi.fn(),
  fillRect: vi.fn(),
  fillText: vi.fn(),
  lineTo: vi.fn(),
  moveTo: vi.fn(),
  setLineDash: vi.fn(),
  stroke: vi.fn(),
  strokeRect: vi.fn(),
});

describe("planner canvas render loop", () => {
  it("renders a frame, smooths telemetry, and cancels the scheduled frame on cleanup", () => {
    const ctx = createContext();
    const scheduleFrame = vi.fn(() => 42);
    const cancelFrame = vi.fn();
    const telemetryRenderRef = { current: { x: 0, y: 0, z: 0, yaw: 0 } };

    const stop = startPlannerCanvasRenderLoop({
      canvasRef: { current: { getContext: () => ctx } },
      telemetryTargetRef: { current: { x: 10, y: 0, z: 0, yaw: 0 } },
      telemetryRenderRef,
      drawStateRef: {
        current: {
          visitEntries: [], chargeEntries: [], plannedVisitEntryMap: new Map(), zoneEntries: [],
          surfaceZones: [], optimizedRoute: [], obstacleTrace: [], obstacleMap: { cells: [] },
          cameraMap: { cells: [], freeCells: [] }, routeBlocked: false, hoveredPointIndex: null,
        },
      },
      scheduleFrame,
      cancelFrame,
    });

    expect(telemetryRenderRef.current.x).toBe(3.5);
    expect(ctx.arc).toHaveBeenCalled();
    expect(scheduleFrame).toHaveBeenCalledOnce();

    stop();
    expect(cancelFrame).toHaveBeenCalledWith(42);
  });
});
