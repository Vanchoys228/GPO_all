const path = require("path");
const defaultConfig = require("../config/runtime-config.cjs");
const coordinateContract = require("../../shared/coordinate-contract.json");
const {createWebStateStore} = require("../artifacts/web-state-store.cjs");
const {createWebotsFileAdapter} = require("../adapters/webots-file-adapter.cjs");
const {createFileTelemetrySource} = require("../adapters/webots-telemetry-source.cjs");
const {createTelemetryNormalizer} = require("../telemetry/normalizer.cjs");
const {createSqliteRepository} = require("../repositories/sqlite-repository.cjs");
const {createGatewayService} = require("../services/gateway-service.cjs");
const {createGatewayServer} = require("../servers/gateway-server.cjs");
const startGatewayProcess = ({config=defaultConfig}={}) => {
  const artifactStore=createWebStateStore({stateDir:config.WEB_STATE_DIR,coordinateContract});
  const adapter=createWebotsFileAdapter({artifactStore,stateDir:config.WEB_STATE_DIR});
  const repository=createSqliteRepository({directory:path.join(config.WEB_STATE_DIR,"gateway-journal")});
  const service=createGatewayService({repository,adapter});
  const telemetrySource=createFileTelemetrySource({stateDir:config.WEB_STATE_DIR,normalizeTelemetry:createTelemetryNormalizer(coordinateContract)});
  return createGatewayServer({host:config.GATEWAY_BIND_HOST,port:config.GATEWAY_PORT,token:config.GATEWAY_TOKEN,service,adapter,telemetrySource,
    ready:async()=>{await repository.ready();await artifactStore.ensureStateDir();},closeResources:()=>repository.close()});
};
module.exports={startGatewayProcess};
