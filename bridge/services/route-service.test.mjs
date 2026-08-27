import { describe, expect, it, vi } from "vitest";
import routeServiceModule from "./route-service.cjs";

const { createRouteService } = routeServiceModule;

describe("route service", () => {
  it("persists a route command through its storage port", async () => {
    const artifactStore = { writeRoute: vi.fn(async () => {}) };
    const service = createRouteService({ artifactStore });
    const result = await service.handle({ type: "route", route: [{ x: 0, y: 0 }] });

    expect(result).toEqual({ handled: true });
    expect(artifactStore.writeRoute).toHaveBeenCalledWith({
      type: "route",
      route: [{ x: 0, y: 0 }],
    });
  });
});
