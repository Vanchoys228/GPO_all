import WebSocket from "ws";
import { describe, expect, it } from "vitest";
import telemetryServerModule from "./telemetry-server.cjs";

const { createTelemetryServer } = telemetryServerModule;

const waitForOpen = (socket) =>
  new Promise((resolve, reject) => {
    socket.once("open", resolve);
    socket.once("error", reject);
  });

const waitForMessage = (socket) =>
  new Promise((resolve, reject) => {
    socket.once("message", (data) => resolve(JSON.parse(data.toString())));
    socket.once("error", reject);
  });

const expectNoMessage = (socket, timeoutMs = 50) =>
  new Promise((resolve, reject) => {
    const cleanup = () => {
      clearTimeout(timer);
      socket.off("message", onMessage);
      socket.off("error", onError);
    };
    const onMessage = (data) => {
      cleanup();
      reject(new Error(`Unexpected message: ${data.toString()}`));
    };
    const onError = (error) => {
      cleanup();
      reject(error);
    };
    const timer = setTimeout(() => {
      cleanup();
      resolve();
    }, timeoutMs);

    socket.once("message", onMessage);
    socket.once("error", onError);
  });

describe("telemetry WebSocket server", () => {
  it("rejects invalid input and continues delivering the next valid telemetry event", async () => {
    const server = createTelemetryServer({
      coordinateContract: { version: 1, telemetry: { messageType: "telemetry" } },
      fileSource: { poll: async () => null },
      host: "127.0.0.1",
      port: 0,
    });
    await new Promise((resolve) => server.wss.once("listening", resolve));
    const port = server.wss.address().port;
    const sender = new WebSocket(`ws://127.0.0.1:${port}`);
    const receiver = new WebSocket(`ws://127.0.0.1:${port}`);

    try {
      await Promise.all([waitForOpen(sender), waitForOpen(receiver)]);

      const noMalformedBroadcast = expectNoMessage(receiver);
      const malformedResponse = waitForMessage(sender);
      sender.send("{");
      expect(await malformedResponse).toMatchObject({
        type: "service.error",
        code: "invalid_json",
      });
      await noMalformedBroadcast;
      expect(server.getStatus()).toEqual({ clientCount: 2, senderCount: 0 });
      expect(server.getLatest()).toBeNull();

      const unsupportedVersionResponse = waitForMessage(sender);
      sender.send(JSON.stringify({
        contractVersion: 99,
        type: "telemetry.event",
        source: "webots-adapter",
        requestId: "telemetry-99",
        timestamp: "2026-01-01T00:00:00.000Z",
        payload: { type: "telemetry" },
      }));
      expect(await unsupportedVersionResponse).toMatchObject({
        type: "service.error",
        code: "unsupported_contract_version",
      });

      const deliveredTelemetry = waitForMessage(receiver);
      sender.send(JSON.stringify({ type: "telemetry", pose: { x: 1, y: 2, z: 0, yaw: 0 } }));
      expect(await deliveredTelemetry).toMatchObject({
        type: "telemetry.event",
        payload: { type: "telemetry", pose: { x: 1, y: 2, z: 0, yaw: 0 } },
      });
      expect(server.getStatus()).toEqual({ clientCount: 2, senderCount: 1 });
    } finally {
      sender.terminate();
      receiver.terminate();
      await server.close();
    }
  });
});
