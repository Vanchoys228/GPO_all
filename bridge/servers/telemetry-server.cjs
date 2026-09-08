const http = require("http");
const {createHealthHandler} = require("./health-handler.cjs");
const WebSocket = require("ws");
const { verifyWebSocketOrigin } = require("../protocol/origin-policy.cjs");
const { createTelemetryService } = require("../services/telemetry-service.cjs");

const safeJsonParse = (text) => {
  try {
    return { payload: JSON.parse(text) };
  } catch {
    return { error: "invalid_json" };
  }
};

const validateTelemetryPayload = async (payload, coordinateContract) => {
  if (!payload || typeof payload !== "object") return "invalid_telemetry_payload";

  if ("contractVersion" in payload) {
    const { unwrapTelemetryEvent, validateContractEnvelope } = await import("../../shared/contracts/index.js");
    const validation = validateContractEnvelope(payload);
    if (!validation.valid) return validation.error;
    if (!unwrapTelemetryEvent(payload)) return "invalid_telemetry_event";
  }

  if ("coordinateContractVersion" in payload &&
      payload.coordinateContractVersion !== coordinateContract.version) {
    return "unsupported_coordinate_contract_version";
  }

  return null;
};

const sendServiceError = (ws, code) => {
  if (ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ type: "service.error", source: "telemetry-service", code }));
};

const createTelemetryServer = ({
  coordinateContract,
  enableMockTelemetry = false,
  filePollMs = 120,
  fileSource,
  host,
  mockIdleMs = 2500,
  port,
  telemetryService: suppliedTelemetryService,
}) => {
  const server = http.createServer(createHealthHandler({service:"telemetry",getStatus:() => ({clientCount:clients.size})}));
  const wss = new WebSocket.Server({ server, maxPayload: 1024 * 1024, verifyClient: verifyWebSocketOrigin });
  const clients = new Set();
  const senders = new Set();
  let telemetryCount = 0;
  let lastRealTelemetryAt = 0;
  let mockPhase = 0;
  const telemetryService = suppliedTelemetryService || createTelemetryService();

  const broadcastTelemetry = async (payload, exclude = null) => {
    telemetryCount += 1;
    const message = await telemetryService.publish(payload);
    broadcast(message, exclude);
  };

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
    ws.on("error", error => console.error("[telemetry] socket error:", error.message));
    clients.add(ws);
    console.log("[telemetry] client connected");
    ws.on("message", async (data) => {
      const text = typeof data === "string" ? data : data.toString();
      const parsed = safeJsonParse(text);
      if (parsed.error) {
        sendServiceError(ws, parsed.error);
        return;
      }
      const validationError = await validateTelemetryPayload(parsed.payload, coordinateContract);
      if (validationError) {
        sendServiceError(ws, validationError);
        return;
      }

      senders.add(ws);
      lastRealTelemetryAt = Date.now();
      const payload = parsed.payload;
      if (telemetryCount % 20 === 1) {
        console.log(`[telemetry] msg ${telemetryCount} size=${text.length}`);
      }
      await broadcastTelemetry(payload, ws);
    });
    ws.on("close", () => {
      clients.delete(ws);
      senders.delete(ws);
      console.log("[telemetry] client disconnected");
    });
  });

  const mockTimer = setInterval(async () => {
    if (!enableMockTelemetry || clients.size === 0) return;
    if (Date.now() - lastRealTelemetryAt < mockIdleMs) return;
    const payload = await telemetryService.publish(buildMockTelemetry());
    for (const client of clients) {
      if (!senders.has(client) && client.readyState === WebSocket.OPEN) client.send(payload);
    }
  }, 250);

  const fileTimer = setInterval(async () => {
    try {
      const telemetry = await fileSource.poll();
      if (!telemetry) return;
      lastRealTelemetryAt = Date.now();
      await broadcastTelemetry(telemetry);
    } catch (error) {
      console.error("[telemetry] failed to poll telemetry source:", error.message);
    }
  }, filePollMs);

  const close = async () => {
    clearInterval(mockTimer);
    clearInterval(fileTimer);
    for (const client of clients) client.terminate();
    await new Promise((resolve) => wss.close(resolve));
    await new Promise(resolve => server.close(resolve));
  };

  server.listen(port,host);
  return {
    broadcast,
    buildMockTelemetry,
    close,
    getStatus: () => ({ clientCount: clients.size, senderCount: senders.size }),
    getLatest: telemetryService.getLatest,
    wss,
  };
};

module.exports = { createTelemetryServer };
