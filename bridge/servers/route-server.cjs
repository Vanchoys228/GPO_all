const WebSocket = require("ws");
const { createRouteService } = require("../services/route-service.cjs");

const safeJsonParse = (text) => {
  try {
    return JSON.parse(text);
  } catch {
    return null;
  }
};

const normalizeIncomingPayload = async (payload) => {
  if (payload?.type !== "route.command") return payload;
  const { unwrapRouteCommand } = await import("../../shared/contracts/index.js");
  return unwrapRouteCommand(payload);
};

const createRouteServer = ({ artifactStore, host, port }) => {
  const wss = new WebSocket.Server({ host, port });
  const uiClients = new Set();
  let controllerConnection = null;
  const routeService = createRouteService({ artifactStore });

  wss.on("connection", (ws, request) => {
    const url = request?.url || "/";
    const isUi = url.startsWith("/ui");
    if (isUi) uiClients.add(ws);
    else controllerConnection = ws;
    console.log(`[route] client connected (${isUi ? "ui" : "controller"})`);

    ws.on("message", async (data) => {
      if (!isUi) return;
      const text = data.toString();
      const payload = await normalizeIncomingPayload(safeJsonParse(text));
      if (!payload) {
        console.error("[route] rejected invalid route contract");
        return;
      }
      try {
        await routeService.handle(payload);
      } catch (error) {
        console.error(`[route] failed to persist ${payload?.type || "unknown"}:`, error.message);
      }

      if (controllerConnection?.readyState === WebSocket.OPEN) {
        controllerConnection.send(JSON.stringify(payload));
      } else {
        console.log("[route] controller websocket not connected; message saved to web_state");
      }
    });

    ws.on("close", () => {
      if (isUi) uiClients.delete(ws);
      if (controllerConnection === ws) controllerConnection = null;
      console.log("[route] client disconnected");
    });
  });

  return {
    close: () => new Promise((resolve) => wss.close(resolve)),
    getStatus: () => ({
      controllerConnected: controllerConnection?.readyState === WebSocket.OPEN,
      uiClientCount: uiClients.size,
    }),
    persistMessage: routeService.handle,
    wss,
  };
};

module.exports = { createRouteServer, normalizeIncomingPayload, safeJsonParse };
