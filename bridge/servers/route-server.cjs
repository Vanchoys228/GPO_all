const WebSocket = require("ws");

const safeJsonParse = (text) => {
  try {
    return JSON.parse(text);
  } catch {
    return null;
  }
};

const createRouteServer = ({ artifactStore, host, port }) => {
  const wss = new WebSocket.Server({ host, port });
  const uiClients = new Set();
  let controllerConnection = null;

  const persistMessage = async (payload) => {
    if (payload?.type === "route") return artifactStore.writeRoute(payload);
    if (payload?.type === "limit_zones") return artifactStore.writeLimitZones(payload);
    if (payload?.type === "surface_zones") return artifactStore.writeSurfaceZones(payload);
    if (payload?.type === "motion_profile") return artifactStore.writeMotionProfile(payload.motion);
    if (payload?.type === "spawn_random_obstacle" || payload?.type === "start_mapping_survey") {
      return artifactStore.writeRuntimeCommand(payload);
    }
    return undefined;
  };

  wss.on("connection", (ws, request) => {
    const url = request?.url || "/";
    const isUi = url.startsWith("/ui");
    if (isUi) uiClients.add(ws);
    else controllerConnection = ws;
    console.log(`[route] client connected (${isUi ? "ui" : "controller"})`);

    ws.on("message", async (data) => {
      if (!isUi) return;
      const text = data.toString();
      const payload = safeJsonParse(text);
      try {
        await persistMessage(payload);
      } catch (error) {
        console.error(`[route] failed to persist ${payload?.type || "unknown"}:`, error.message);
      }

      if (controllerConnection?.readyState === WebSocket.OPEN) {
        controllerConnection.send(text);
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
    persistMessage,
    wss,
  };
};

module.exports = { createRouteServer, safeJsonParse };
