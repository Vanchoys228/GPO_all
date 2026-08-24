const WebSocket = require("ws");

const createTelemetryServer = ({
  coordinateContract,
  enableMockTelemetry = false,
  filePollMs = 120,
  fileSource,
  host,
  mockIdleMs = 2500,
  port,
}) => {
  const wss = new WebSocket.Server({ host, port });
  const clients = new Set();
  const senders = new Set();
  let telemetryCount = 0;
  let lastRealTelemetryAt = 0;
  let mockPhase = 0;

  const broadcast = (payload, exclude = null) => {
    for (const client of clients) {
      if (client === exclude) continue;
      if (client.readyState === WebSocket.OPEN) client.send(payload);
    }
  };

  const buildMockTelemetry = () => {
    mockPhase += 0.2;
    const radius = 3.8;
    const x = radius * Math.cos(mockPhase);
    const y = radius * Math.sin(mockPhase);
    const z = 0;
    const yaw = mockPhase + Math.PI / 2;
    return {
      type: coordinateContract.telemetry.messageType,
      coordinateContractVersion: coordinateContract.version,
      pose: { x, y, z, yaw },
      x,
      y,
      z,
      yaw,
    };
  };

  wss.on("connection", (ws) => {
    clients.add(ws);
    console.log("[telemetry] client connected");
    ws.on("message", (data) => {
      senders.add(ws);
      lastRealTelemetryAt = Date.now();
      const text = typeof data === "string" ? data : data.toString();
      telemetryCount += 1;
      if (telemetryCount % 20 === 1) {
        console.log(`[telemetry] msg ${telemetryCount} size=${text.length}`);
      }
      broadcast(text, ws);
    });
    ws.on("close", () => {
      clients.delete(ws);
      senders.delete(ws);
      console.log("[telemetry] client disconnected");
    });
  });

  const mockTimer = setInterval(() => {
    if (!enableMockTelemetry || clients.size === 0) return;
    if (Date.now() - lastRealTelemetryAt < mockIdleMs) return;
    const payload = JSON.stringify(buildMockTelemetry());
    for (const client of clients) {
      if (!senders.has(client) && client.readyState === WebSocket.OPEN) client.send(payload);
    }
  }, 250);

  const fileTimer = setInterval(async () => {
    try {
      const telemetry = await fileSource.poll();
      if (!telemetry) return;
      lastRealTelemetryAt = Date.now();
      broadcast(JSON.stringify(telemetry));
    } catch (error) {
      console.error("[telemetry] failed to poll robot_state.json:", error.message);
    }
  }, filePollMs);

  const close = async () => {
    clearInterval(mockTimer);
    clearInterval(fileTimer);
    for (const client of clients) client.terminate();
    await new Promise((resolve) => wss.close(resolve));
  };

  return {
    broadcast,
    buildMockTelemetry,
    close,
    getStatus: () => ({ clientCount: clients.size, senderCount: senders.size }),
    wss,
  };
};

module.exports = { createTelemetryServer };
