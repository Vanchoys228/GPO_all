import { describe, expect, it, vi } from "vitest";
import localStackModule from "./local-stack.cjs";

const { startLocalStack } = localStackModule;

describe("local service stack", () => {
  it("starts all services and stops them together", async () => {
    const telemetry = { close: vi.fn(async () => {}) };
    const route = { close: vi.fn(async () => {}) };
    const planning = { close: vi.fn((done) => done()) };
    const startTelemetryProcess = vi.fn(() => telemetry);
    const startRouteProcess = vi.fn(() => route);
    const startPlanningProcess = vi.fn(() => planning);

    const stack = startLocalStack({
      enableMockTelemetry: true,
      startTelemetryProcess,
      startRouteProcess,
      startPlanningProcess,
    });

    expect(startTelemetryProcess).toHaveBeenCalledWith({ enableMockTelemetry: true });
    expect(startRouteProcess).toHaveBeenCalledOnce();
    expect(startPlanningProcess).toHaveBeenCalledOnce();

    await stack.stop();

    expect(telemetry.close).toHaveBeenCalledOnce();
    expect(route.close).toHaveBeenCalledOnce();
    expect(planning.close).toHaveBeenCalledOnce();
  });
});
