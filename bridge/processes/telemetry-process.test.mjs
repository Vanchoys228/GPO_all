import {expect,it,vi} from "vitest";
import module from "./telemetry-process.cjs";
it("uses gateway telemetry without accessing simulator files",async()=>{
  const client={ready:vi.fn(),telemetry:vi.fn(async()=>({revision:1,fresh:true,telemetry:{pose:{x:1}}})),close:vi.fn()};
  const createClient=vi.fn(()=>client);
  const createTelemetryServer=vi.fn(options=>options);
  const options=module.startTelemetryProcess({config:{GATEWAY_URL:"http://robot:9004",GATEWAY_TOKEN:"test",BRIDGE_HOST:"127.0.0.1",TELEMETRY_PORT:9001},createClient,createTelemetryServer});
  expect(createClient).toHaveBeenCalledWith({baseUrl:"http://robot:9004",token:"test"});
  expect(await options.fileSource.poll()).toEqual({pose:{x:1}});
  expect(await options.fileSource.poll()).toBeNull();
  options.fileSource.close();expect(client.close).toHaveBeenCalledOnce();
});
