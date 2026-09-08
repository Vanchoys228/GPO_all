const path = require("path");
const {createWebotsFileAdapter:defaultCreateSimulatorAdapter} = require("../adapters/webots-file-adapter.cjs");
const {createMissionRepository:defaultCreateMissionRepository} = require("../repositories/mission-repository.cjs");
const {createMissionService} = require("../services/mission-service.cjs");
const {createRouteService} = require("../services/route-service.cjs");
const defaultConfig = require("../config/runtime-config.cjs");
const defaultCoordinateContract = require("../../shared/coordinate-contract.json");
const { createWebStateStore: defaultCreateWebStateStore } = require("../artifacts/web-state-store.cjs");
const { createRouteServer: defaultCreateRouteServer } = require("../servers/route-server.cjs");

const startRouteProcess = ({
  config = defaultConfig,
  coordinateContract = defaultCoordinateContract,
  createWebStateStore = defaultCreateWebStateStore,
  createRouteServer = defaultCreateRouteServer,
  createSimulatorAdapter = defaultCreateSimulatorAdapter,
  createMissionRepository = defaultCreateMissionRepository,
} = {}) => {
  const artifactStore = createWebStateStore({
    coordinateContract,
    stateDir: config.WEB_STATE_DIR,
  });

  const adapter = createSimulatorAdapter({artifactStore,stateDir:config.WEB_STATE_DIR});
  const repository = createMissionRepository({directory:config.MISSION_STATE_DIR || path.join(config.WEB_STATE_DIR,"missions")});
  const missionService = createMissionService({repository,adapter});
  const routeService = createRouteService({missionService,adapter});
  return createRouteServer({
    routeService,
    missionService,
    ready:async () => {await artifactStore.ensureStateDir();await repository.ready();},
    host: config.BRIDGE_HOST,
    port: config.ROUTE_PORT,
  });
};

module.exports = { startRouteProcess };
