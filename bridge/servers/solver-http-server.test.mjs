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
});
