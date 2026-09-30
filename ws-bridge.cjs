const {
  ROUTE_WS_URL,
  TELEMETRY_WS_URL,
} = require("./bridge/config/runtime-config.cjs");
const { startLocalStack } = require("./bridge/processes/local-stack.cjs");

const enableMockTelemetry = process.env.MOCK_TELEMETRY === "1";
const stack = startLocalStack({ enableMockTelemetry });

console.log(`[bridge] telemetry ${TELEMETRY_WS_URL}, route ${ROUTE_WS_URL}`);
if (enableMockTelemetry) {
  console.log("[bridge] mock telemetry is enabled (MOCK_TELEMETRY=1)");
}

let shuttingDown = false;
const shutdown = async (signal) => {
  if (shuttingDown) return;
  shuttingDown = true;
  console.log(`[bridge] stopping (${signal})`);
  await stack.stop();
  process.exitCode = 0;
};

process.once("SIGINT", () => void shutdown("SIGINT"));
process.once("SIGTERM", () => void shutdown("SIGTERM"));
