import {expect,it,vi} from "vitest";
import module from "./route-process.cjs";
it("composes missions with private storage and a network simulator port", async()=>{
  const adapter={ready:vi.fn(),close:vi.fn()};
  const repository={ready:vi.fn(),close:vi.fn()};
  const createSimulatorAdapter=vi.fn(()=>adapter);
  const createMissionRepository=vi.fn(()=>repository);
  const close=vi.fn(async()=>{});
  const createRouteServer=vi.fn(()=>({close}));
  const server=module.startRouteProcess({config:{GATEWAY_URL:"http://robot:9004",GATEWAY_TOKEN:"test",MISSION_STATE_DIR:"/missions",BRIDGE_HOST:"127.0.0.1",ROUTE_PORT:9002},createSimulatorAdapter,createMissionRepository,createRouteServer});
  expect(createSimulatorAdapter).toHaveBeenCalledWith({baseUrl:"http://robot:9004",token:"test"});
  expect(createMissionRepository).toHaveBeenCalledWith({directory:"/missions"});
  await createRouteServer.mock.calls[0][0].ready();
  expect(adapter.ready).toHaveBeenCalledOnce();
  await server.close();expect(repository.close).toHaveBeenCalledOnce();expect(close).toHaveBeenCalledOnce();
});
