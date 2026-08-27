import { describe, expect, it, vi } from "vitest";
import routeProcessModule from "./route-process.cjs";

const { startRouteProcess } = routeProcessModule;

describe("route process", () => {
  it("starts the route websocket server with its own state store", () => {
    const server = { marker: "route-server" };
    const createWebStateStore = vi.fn(() => ({ marker: "state-store" }));
    const createRouteServer = vi.fn(() => server);
    const config = { BRIDGE_HOST: "127.0.0.1", ROUTE_PORT: 9002, WEB_STATE_DIR: "/state" };
    const coordinateContract = { version: 1 };

    const result = startRouteProcess({
      config,
      coordinateContract,
      createWebStateStore,
      createRouteServer,
    });

    expect(createWebStateStore).toHaveBeenCalledWith({ coordinateContract, stateDir: "/state" });
    expect(createRouteServer).toHaveBeenCalledWith({
      artifactStore: { marker: "state-store" },
      host: "127.0.0.1",
      port: 9002,
    });
    expect(result).toBe(server);
  });
});
