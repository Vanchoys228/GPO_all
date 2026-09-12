const {createTelemetryService} = require("../services/telemetry-service.cjs");
const defaultConfig = require("../config/runtime-config.cjs");
const defaultCoordinateContract = require("../../shared/coordinate-contract.json");
const {createTelemetryServer:defaultCreateTelemetryServer} = require("../servers/telemetry-server.cjs");
const {createGatewayClient,createGatewayTelemetrySource} = require("../adapters/gateway-http-client.cjs");
const startTelemetryProcess = ({config=defaultConfig,coordinateContract=defaultCoordinateContract,
  createTelemetryServer=defaultCreateTelemetryServer,createClient=createGatewayClient,
  enableMockTelemetry=process.env.MOCK_TELEMETRY==="1"}={})=>{
  const client=createClient({baseUrl:config.GATEWAY_URL,token:config.GATEWAY_TOKEN});
  const source=createGatewayTelemetrySource({client});
  return createTelemetryServer({coordinateContract,enableMockTelemetry,fileSource:source,
    ready:client.ready,telemetryService:createTelemetryService(),
    host:config.TELEMETRY_BIND_HOST || config.BRIDGE_HOST,port:config.TELEMETRY_PORT});
};
module.exports={startTelemetryProcess};
