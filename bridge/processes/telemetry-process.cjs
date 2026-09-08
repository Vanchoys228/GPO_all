const {createTelemetryService} = require("../services/telemetry-service.cjs");
const defaultConfig = require("../config/runtime-config.cjs");
const defaultCoordinateContract = require("../../shared/coordinate-contract.json");
const { createTelemetryServer: defaultCreateTelemetryServer } = require("../servers/telemetry-server.cjs");
const { createFileTelemetrySource: defaultCreateFileTelemetrySource } = require("../adapters/webots-telemetry-source.cjs");
const { createTelemetryNormalizer: defaultCreateTelemetryNormalizer } = require("../telemetry/normalizer.cjs");

const startTelemetryProcess = ({
  config = defaultConfig,
  coordinateContract = defaultCoordinateContract,
  createTelemetryServer = defaultCreateTelemetryServer,
  createFileTelemetrySource = defaultCreateFileTelemetrySource,
  createTelemetryNormalizer = defaultCreateTelemetryNormalizer,
  enableMockTelemetry = process.env.MOCK_TELEMETRY === "1",
} = {}) => {
  const normalizer = createTelemetryNormalizer(coordinateContract);
  const fileSource = createFileTelemetrySource({
    normalizeTelemetry: normalizer,
    stateDir: config.WEB_STATE_DIR,
  });

  return createTelemetryServer({
    coordinateContract,
    enableMockTelemetry,
    fileSource,
    telemetryService:createTelemetryService(),
    host: config.BRIDGE_HOST,
    port: config.TELEMETRY_PORT,
  });
};

module.exports = { startTelemetryProcess };
