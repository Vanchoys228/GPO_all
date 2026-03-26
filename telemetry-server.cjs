const WebSocket = require("ws");
const coordinateContract = require("./shared/coordinate-contract.json");
const {
  BRIDGE_HOST,
  TELEMETRY_PORT,
  TELEMETRY_WS_URL,
} = require("./bridge-config.cjs");

const wss = new WebSocket.Server({ host: BRIDGE_HOST, port: TELEMETRY_PORT });
let phase = 0;

console.log(`Mock telemetry server running on ${TELEMETRY_WS_URL}`);

setInterval(() => {
  phase += 0.15;

  const x = Number((Math.cos(phase) * 4).toFixed(3));
  const y = Number((Math.sin(phase) * 4).toFixed(3));
  const z = 0;
  const yaw = Number((phase + Math.PI / 2).toFixed(6));

  const data = {
    type: coordinateContract.telemetry.messageType,
    coordinateContractVersion: coordinateContract.version,
    pose: { x, y, z, yaw },
    x,
    y,
    z,
    yaw,
    simulationTime: Date.now() / 1000,
  };

  for (const client of wss.clients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(data));
    }
  }
}, 500);
