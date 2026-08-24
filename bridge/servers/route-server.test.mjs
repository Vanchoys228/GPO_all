import WebSocket from "ws";
import { describe, expect, it, vi } from "vitest";
import routeServerModule from "./route-server.cjs";

const { createRouteServer } = routeServerModule;

const waitForOpen = (socket) =>
  new Promise((resolve, reject) => {
    socket.once("open", resolve);
    socket.once("error", reject);
  });

const waitForMessage = (socket) =>
  new Promise((resolve, reject) => {
    socket.once("message", (data) => resolve(data.toString()));
    socket.once("error", reject);
  });

describe("route WebSocket server", () => {
  it("persists UI messages and forwards them to the controller", async () => {
    const artifactStore = {
      writeRoute: vi.fn(async () => {}),
      writeLimitZones: vi.fn(async () => {}),
      writeSurfaceZones: vi.fn(async () => {}),
      writeMotionProfile: vi.fn(async () => {}),
      writeRuntimeCommand: vi.fn(async () => {}),
    };
    const server = createRouteServer({ artifactStore, host: "127.0.0.1", port: 0 });
    await new Promise((resolve) => server.wss.once("listening", resolve));
    const port = server.wss.address().port;
    const controller = new WebSocket(`ws://127.0.0.1:${port}`);
    const ui = new WebSocket(`ws://127.0.0.1:${port}/ui`);

    try {
      await Promise.all([waitForOpen(controller), waitForOpen(ui)]);
      const payload = JSON.stringify({ type: "route", route: [{ x: 0, y: 0 }] });
      const forwardedMessage = waitForMessage(controller);
      ui.send(payload);

      expect(await forwardedMessage).toBe(payload);
      await vi.waitFor(() => expect(artifactStore.writeRoute).toHaveBeenCalledOnce());
      expect(server.getStatus()).toEqual({ controllerConnected: true, uiClientCount: 1 });
    } finally {
      controller.terminate();
      ui.terminate();
      await server.close();
    }
  });
});
