import { afterEach, describe, expect, it } from "vitest";
import serverModule from "./solver-http-server.cjs";

const { createSolverHttpServer } = serverModule;

let activeServer = null;

afterEach(async () => {
  if (!activeServer) return;
  await new Promise((resolve) => activeServer.close(resolve));
  activeServer = null;
});

const listenForTest = async (nativeSolver) => {
  const instance = createSolverHttpServer({
    coordinateContract: { version: 1 },
    host: "127.0.0.1",
    nativeSolver,
    port: 0,
    solverPath: "/test/solver",
  });
  activeServer = instance.server;
  await new Promise((resolve) => activeServer.listen(0, "127.0.0.1", resolve));
  const address = activeServer.address();
  return `http://127.0.0.1:${address.port}`;
};

const solveRequest = (baseUrl, points = [{ x: 0, y: 0 }, { x: 1, y: 0 }]) =>
  fetch(`${baseUrl}/api/solve-route`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      points,
      task: "tsp",
      algorithm: { key: "ga_tabu", params: {} },
    }),
  });

describe("solver HTTP server", () => {
  it("reports health without requiring a solver run", async () => {
    const baseUrl = await listenForTest({
      solverExists: async () => false,
      run: async () => {
        throw new Error("must not run");
      },
    });

    const response = await fetch(`${baseUrl}/health`);
    expect(response.status).toBe(200);
    expect(await response.json()).toEqual({
      ok: true,
      coordinateContractVersion: 1,
      solverAvailable: false,
      solverPath: "/test/solver",
    });
  });

  it("keeps the existing solve-route response contract", async () => {
    const baseUrl = await listenForTest({
      solverExists: async () => true,
      run: async () => ({
        length: 1,
        closed: true,
        order: [0, 1],
        route: [
          { x: 0, y: 0 },
          { x: 1, y: 0 },
        ],
      }),
    });

    const response = await fetch(`${baseUrl}/api/solve-route`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        points: [
          { x: 0, y: 0 },
          { x: 1, y: 0 },
        ],
        task: "tsp",
        algorithm: { key: "ga_tabu", params: {} },
      }),
    });

    expect(response.status).toBe(200);
    expect(await response.json()).toMatchObject({
      ok: true,
      task: "tsp",
      algorithm: "ga_tabu",
      route: [
        { x: 0, y: 0 },
        { x: 1, y: 0 },
      ],
    });
  });

  it("returns 400 without invoking the solver for oversized routes", async () => {
    let runCount = 0;
    const baseUrl = await listenForTest({
      solverExists: async () => true,
      run: async () => {
        runCount += 1;
      },
    });
    const points = Array.from({ length: 1001 }, (_, index) => ({ x: index, y: 0 }));

    const response = await solveRequest(baseUrl, points);

    expect(response.status).toBe(400);
    expect(runCount).toBe(0);
  });

  it("allows only two native solver runs at once", async () => {
    const releases = [];
    let runCount = 0;
    const baseUrl = await listenForTest({
      solverExists: async () => true,
      run: async () => {
        runCount += 1;
        return new Promise((resolve) => releases.push(() => resolve({
          length: 1,
          closed: true,
          order: [0, 1],
          route: [{ x: 0, y: 0 }, { x: 1, y: 0 }],
        })));
      },
    });

    const first = solveRequest(baseUrl);
    const second = solveRequest(baseUrl);
    while (runCount < 2) await new Promise((resolve) => setTimeout(resolve, 5));

    const thirdRequest = solveRequest(baseUrl);
    await new Promise((resolve) => setTimeout(resolve, 25));
    releases.forEach((release) => release());
    const third = await thirdRequest;
    expect(third.status).toBe(503);
    expect(runCount).toBe(2);

    expect((await first).status).toBe(200);
    expect((await second).status).toBe(200);
  });
});
