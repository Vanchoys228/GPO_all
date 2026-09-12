import {expect,it,vi} from "vitest";
import client from "./gateway-http-client.cjs";
it("retries a lost response with exactly the same version and ID",async()=>{
  const fetchImpl=vi.fn().mockRejectedValueOnce(new Error("connection reset")).mockResolvedValue(new Response(JSON.stringify({version:1,ok:true,payload:{accepted:true}})));
  const adapter=client.createGatewayClient({baseUrl:"http://gateway:9004",fetchImpl});
  await adapter.submit({commandId:"retry-1"});
  expect(fetchImpl).toHaveBeenCalledTimes(2);
  expect(fetchImpl.mock.calls[0][1].body).toBe(fetchImpl.mock.calls[1][1].body);
  expect(JSON.parse(fetchImpl.mock.calls[0][1].body)).toMatchObject({version:1,requestId:"retry-1"});
});
it("does not retry conflicts and rejects incompatible responses",async()=>{
  const fetchImpl=vi.fn(async()=>new Response(JSON.stringify({version:1,ok:false,code:"id_conflict",error:"conflict"}),{status:409}));
  const adapter=client.createGatewayClient({baseUrl:"http://gateway:9004",fetchImpl});
  await expect(adapter.submit({commandId:"same"})).rejects.toMatchObject({statusCode:409});
  expect(fetchImpl).toHaveBeenCalledOnce();
});
it("bounds requests with an abort signal",async()=>{
  const fetchImpl=vi.fn((url,{signal})=>new Promise((resolve,reject)=>signal.addEventListener("abort",()=>reject(new Error("timeout")),{once:true})));
  const adapter=client.createGatewayClient({baseUrl:"http://gateway:9004",fetchImpl,timeoutMs:10,retries:0});
  await expect(adapter.ready()).rejects.toMatchObject({code:"gateway_unavailable"});
});
