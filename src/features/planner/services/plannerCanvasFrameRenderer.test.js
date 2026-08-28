import { describe, expect, it, vi } from "vitest";
import { renderPlannerCanvasFrame } from "./plannerCanvasFrameRenderer";

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

describe("planner canvas frame renderer", () => {
  it("renders the background and robot for an empty planner state", () => {
    const ctx = createContext();

    renderPlannerCanvasFrame(
      ctx,
      {
        visitEntries: [],
        chargeEntries: [],
        plannedVisitEntryMap: new Map(),
        zoneEntries: [],
        surfaceZones: [],
        optimizedRoute: [],
        obstacleTrace: [],
        obstacleMap: { cells: [] },
        cameraMap: { cells: [], freeCells: [] },
        routeBlocked: false,
        hoveredPointIndex: null,
      },
      { x: 0, y: 0, yaw: 0 }
    );

    expect(ctx.fillRect).toHaveBeenCalled();
    expect(ctx.arc).toHaveBeenCalledWith(expect.any(Number), expect.any(Number), 11, 0, Math.PI * 2);
  });
});
