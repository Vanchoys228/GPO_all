const coordinateContract = require("./shared/coordinate-contract.json");
const { createWebStateStore } = require("./bridge/artifacts/web-state-store.cjs");
const {
  BRIDGE_HOST,
  ROUTE_PORT,
  ROUTE_WS_URL,
  SOLVER_PATH,
  SOLVER_PORT,
  TELEMETRY_PORT,
  TELEMETRY_WS_URL,
  WEB_STATE_DIR,
} = require("./bridge/config/runtime-config.cjs");
const { createRouteServer } = require("./bridge/servers/route-server.cjs");
const { createSolverHttpServer } = require("./bridge/servers/solver-http-server.cjs");
const { createTelemetryServer } = require("./bridge/servers/telemetry-server.cjs");
const { createNativeSolver } = require("./bridge/solver/native-solver.cjs");
const { createFileTelemetrySource } = require("./bridge/telemetry/file-source.cjs");
const { createTelemetryNormalizer } = require("./bridge/telemetry/normalizer.cjs");

const enableMockTelemetry = process.env.MOCK_TELEMETRY === "1";
const nativeSolver = createNativeSolver({ solverPath: SOLVER_PATH });
const artifactStore = createWebStateStore({
  coordinateContract,
  stateDir: WEB_STATE_DIR,
});
const normalizeTelemetry = createTelemetryNormalizer(coordinateContract);
const fileTelemetrySource = createFileTelemetrySource({
  normalizeTelemetry,
  stateDir: WEB_STATE_DIR,
});

const telemetryServer = createTelemetryServer({
  coordinateContract,
  enableMockTelemetry,
  fileSource: fileTelemetrySource,
  host: BRIDGE_HOST,
  port: TELEMETRY_PORT,
});
const routeServer = createRouteServer({
  artifactStore,
  host: BRIDGE_HOST,
  port: ROUTE_PORT,
});
const solverHttpServer = createSolverHttpServer({
  coordinateContract,
  host: BRIDGE_HOST,
  nativeSolver,
  port: SOLVER_PORT,
  solverPath: SOLVER_PATH,
});
solverHttpServer.start();

console.log(`[bridge] telemetry ${TELEMETRY_WS_URL}, route ${ROUTE_WS_URL}`);
if (enableMockTelemetry) {
  console.log("[bridge] mock telemetry is enabled (MOCK_TELEMETRY=1)");
}

let shuttingDown = false;
const shutdown = async (signal) => {
  if (shuttingDown) return;
  shuttingDown = true;
  console.log(`[bridge] stopping (${signal})`);
  await Promise.allSettled([
    telemetryServer.close(),
    routeServer.close(),
    new Promise((resolve) => solverHttpServer.server.close(resolve)),
  ]);
  process.exitCode = 0;
};

process.once("SIGINT", () => void shutdown("SIGINT"));
process.once("SIGTERM", () => void shutdown("SIGTERM"));
