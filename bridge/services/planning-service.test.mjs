import { describe, expect, it, vi } from "vitest";
import planningServiceModule from "./planning-service.cjs";
import { createPlanningRequest } from "../../shared/contracts/index.js";

const { createPlanningService } = planningServiceModule;

describe("planning service", () => {
  it("applies energy constraints on the server using the submitted scene", async () => {
    const nativeSolver = { run: async () => ({route:[{x:0,y:0},{x:1,y:0},{x:1,y:1}],length:2}) };
    const service = createPlanningService({nativeSolver});
    await expect(service.solve({points:[{x:0,y:0},{x:1,y:0},{x:1,y:1}],scene:{polygons:[],surfaceZones:[],chargingStations:[],motion:{batteryRange:2.2}}})).rejects.toThrow(/запас|достижим|range/);
  });
  it("returns a final route and metrics with a server scene revision", async () => {
    const nativeSolver = { run: async () => ({route:[{x:0,y:0},{x:1,y:0}],length:1}) };
    const result = await createPlanningService({nativeSolver}).solve({points:[{x:0,y:0},{x:1,y:0}],scene:{polygons:[],surfaceZones:[],chargingStations:[],motion:{batteryRange:100}}});
    expect(result.planning.routeEnergy).toBeGreaterThan(0);
    expect(result.sceneRevision).toMatch(/^[a-f0-9]{64}$/);
    expect(result.seedRoute).toHaveLength(2);
  });
  it("rejects malformed scene snapshots before starting the solver", async () => {
    const run = vi.fn();
    await expect(createPlanningService({nativeSolver:{run}}).solve({points:[{x:0,y:0}],scene:{surfaceZones:"bad"}})).rejects.toThrow();
    expect(run).not.toHaveBeenCalled();
  });
  it("normalizes a request and delegates it to the native solver", async () => {
    const nativeSolver = { run: vi.fn(async () => ({ length: 1, closed: true, order: [0], route: [{ x: 0, y: 0 }] })) };
    const service = createPlanningService({ nativeSolver });

    const result = await service.solve({
      points: [{ x: 0, y: 0 }],
      task: "tsp",
      algorithm: { key: "ga_tabu", params: {} },
    });

    expect(result).toMatchObject({ ok: true, task: "tsp", algorithm: "ga_tabu", route: [{ x: 0, y: 0 }] });
    expect(nativeSolver.run).toHaveBeenCalledOnce();
  });

  it("accepts a versioned planning request without changing the solver input", async () => {
    const nativeSolver = { run: vi.fn(async () => ({ length: 1, closed: true, order: [0], route: [{ x: 0, y: 0 }] })) };
    const service = createPlanningService({ nativeSolver });
    const request = createPlanningRequest({
      source: "frontend",
      requestId: "planning-42",
      timestamp: "2026-01-01T00:00:00.000Z",
      payload: {
        points: [{ x: 0, y: 0 }],
        task: "tsp",
        algorithm: { key: "ga_tabu", params: {} },
      },
    });

    await service.solve(request);

    expect(nativeSolver.run).toHaveBeenCalledWith(expect.objectContaining({
      points: [{ x: 0, y: 0 }],
      taskKey: "tsp",
      algorithmKey: "ga_tabu",
    }));
  });
});
