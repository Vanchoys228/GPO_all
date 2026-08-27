const defaultConfig = require("../config/runtime-config.cjs");
const defaultCoordinateContract = require("../../shared/coordinate-contract.json");
const { createWebStateStore: defaultCreateWebStateStore } = require("../artifacts/web-state-store.cjs");
const { createRouteServer: defaultCreateRouteServer } = require("../servers/route-server.cjs");

const startRouteProcess = ({
  config = defaultConfig,
  coordinateContract = defaultCoordinateContract,
  createWebStateStore = defaultCreateWebStateStore,
  createRouteServer = defaultCreateRouteServer,
} = {}) => {
  const artifactStore = createWebStateStore({
    coordinateContract,
    stateDir: config.WEB_STATE_DIR,
  });

  return createRouteServer({
    artifactStore,
    host: config.BRIDGE_HOST,
    port: config.ROUTE_PORT,
  });
};

module.exports = { startRouteProcess };
