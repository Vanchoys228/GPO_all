import { describe, expect, it, vi } from "vitest";
import planningServiceModule from "./planning-service.cjs";
import { createPlanningRequest } from "../../shared/contracts/index.js";

const { createPlanningService } = planningServiceModule;

describe("planning service", () => {
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
