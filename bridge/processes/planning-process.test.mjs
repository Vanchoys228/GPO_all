import { describe, expect, it, vi } from "vitest";
import planningProcessModule from "./planning-process.cjs";

const { startPlanningProcess } = planningProcessModule;

describe("planning process", () => {
  it("starts the planning HTTP server with its own runtime dependencies", () => {
    const server = { marker: "planning-server" };
    const start = vi.fn(() => server);
    const createNativeSolver = vi.fn(() => ({ marker: "native-solver" }));
    const createSolverHttpServer = vi.fn(() => ({ start }));
    const config = { BRIDGE_HOST: "127.0.0.1", SOLVER_PORT: 9003, SOLVER_PATH: "/solver" };
    const coordinateContract = { version: 1 };

    const result = startPlanningProcess({
      config,
      coordinateContract,
      createNativeSolver,
      createSolverHttpServer,
    });

    expect(createNativeSolver).toHaveBeenCalledWith({ solverPath: "/solver" });
    expect(createSolverHttpServer).toHaveBeenCalledWith({
      coordinateContract,
      host: "127.0.0.1",
      nativeSolver: { marker: "native-solver" },
      port: 9003,
      solverPath: "/solver",
    });
    expect(start).toHaveBeenCalledOnce();
    expect(result).toBe(server);
  });
});
