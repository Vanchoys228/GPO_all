import { afterEach, describe, expect, it, vi } from "vitest";
import { solveRouteWithNativeAlgorithm } from "./routeAlgorithms";
import { createPlanningResult } from "../../shared/contracts/index.js";

afterEach(() => {
  vi.unstubAllGlobals();
});

describe("native route solver client", () => {
  it("sends a versioned planning request and preserves the solved route", async () => {
    const fetchMock = vi.fn(async () => new Response(JSON.stringify(createPlanningResult({
      source: "planning-service",
      requestId: "planning-result-1",
      payload: {
        ok: true,
        route: [{ x: "1", y: "2" }],
      },
    })), { status: 200 }));
    vi.stubGlobal("fetch", fetchMock);

    const result = await solveRouteWithNativeAlgorithm(
      [{ x: 1, y: 2 }],
      "ga_tabu",
      {},
      "tsp"
    );

    const request = JSON.parse(fetchMock.mock.calls[0][1].body);
    expect(request).toMatchObject({
      contractVersion: 1,
      type: "planning.request",
      source: "planner-frontend",
      payload: {
        points: [{ x: 1, y: 2 }],
        algorithm: { key: "ga_tabu", params: {} },
        task: "tsp",
      },
    });
    expect(result.route).toEqual([{ x: 1, y: 2 }]);
  });
});
