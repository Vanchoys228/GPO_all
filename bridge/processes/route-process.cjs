const {createGatewayClient} = require("../adapters/gateway-http-client.cjs");
const {createSqliteRepository} = require("../repositories/sqlite-repository.cjs");
const {createMissionService} = require("../services/mission-service.cjs");
const {createRouteService} = require("../services/route-service.cjs");
const defaultConfig = require("../config/runtime-config.cjs");
const {createRouteServer:defaultCreateRouteServer} = require("../servers/route-server.cjs");
const startRouteProcess = ({config=defaultConfig,createRouteServer=defaultCreateRouteServer,
  createSimulatorAdapter=createGatewayClient,createMissionRepository=createSqliteRepository}={}) => {
  const adapter=createSimulatorAdapter({baseUrl:config.GATEWAY_URL,token:config.GATEWAY_TOKEN});
  const repository=createMissionRepository({directory:config.MISSION_STATE_DIR});
  const missionService=createMissionService({repository,adapter});
  const routeService=createRouteService({missionService});
  let inFlight=false, stopped=false, tickPromise;
  const tick=async()=>{
    if(inFlight || stopped)return;
    inFlight=true;
    try {await repository.ready();await missionService.reconcile();}
    catch(error){console.error(JSON.stringify({service:"missions",code:"reconcile_failed",error:error.message}));}
    finally {inFlight=false;}
  };
  const timer=setInterval(()=>{tickPromise=tick();},1000);
  timer.unref();
  const server=createRouteServer({routeService,missionService,
    ready:async()=>{await repository.ready();await adapter.ready();},
    host:config.ROUTE_BIND_HOST || config.BRIDGE_HOST,port:config.ROUTE_PORT});
  const close=server.close;
  server.close=async()=>{stopped=true;clearInterval(timer);await tickPromise;await close();adapter.close?.();await repository.close?.();};
  return server;
};
module.exports={startRouteProcess};
