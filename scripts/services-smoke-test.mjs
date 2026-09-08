import {spawn} from "node:child_process";
import {createServer} from "node:net";
import {mkdtemp,readFile,writeFile,rm} from "node:fs/promises";
import {once} from "node:events";
import os from "node:os";
import path from "node:path";
import assert from "node:assert/strict";
import WebSocket from "ws";
import {createRouteCommand} from "../shared/contracts/index.js";

const root = path.resolve(import.meta.dirname,"..");
const directory = await mkdtemp(path.join(os.tmpdir(),"gpo-services-"));
const children = [];
let socket;
const port = async () => {
  const server = createServer();server.listen(0,"127.0.0.1");await once(server,"listening");
  const value = server.address().port;await new Promise(resolve => server.close(resolve));return value;
};
const [planningPort,routePort,telemetryPort] = await Promise.all([port(),port(),port()]);
const env = {...process.env,BRIDGE_BIND_HOST:"127.0.0.1",SOLVER_PORT:String(planningPort),ROUTE_PORT:String(routePort),TELEMETRY_PORT:String(telemetryPort),WEB_STATE_DIR:directory,MISSION_STATE_DIR:path.join(directory,"missions")};
const start = name => {
  const child = spawn(process.execPath,[`bridge/processes/run-${name}-service.cjs`],{cwd:root,env,stdio:["ignore","pipe","pipe"],windowsHide:true});
  child.logs="";child.stdout.on("data",data => child.logs+=data);child.stderr.on("data",data => child.logs+=data);children.push(child);return child;
};
const stop = async child => {if(child.exitCode !== null || child.signalCode) return;const exited=once(child,"exit");child.kill();await exited;};
const waitReady = async (portNumber,child) => {
  for(let i=0;i<100;i++) {
    if(child.exitCode !== null) throw new Error(child.logs);
    try {const response=await fetch(`http://127.0.0.1:${portNumber}/ready`,{signal:AbortSignal.timeout(1000)});if(response.ok)return;} catch { /* startup */ }
    await new Promise(resolve => setTimeout(resolve,100));
  }
  throw new Error(`Service not ready: ${child.logs}`);
};
const send = command => new Promise((resolve,reject) => {
  const timer=setTimeout(() => {socket.off("message",receive);reject(new Error("ACK timeout"));},10000);
  const receive=data => {const response=JSON.parse(data.toString());if(response.requestId!==command.requestId)return;clearTimeout(timer);socket.off("message",receive);resolve(response);};
  socket.on("message",receive);socket.send(JSON.stringify(command));
});
const getMission = async () => (await fetch(`http://127.0.0.1:${routePort}/api/missions/smoke-mission`)).json();
try {
  const planning=start("planning"), telemetry=start("telemetry");let missions=start("route");
  await Promise.all([waitReady(planningPort,planning),waitReady(routePort,missions),waitReady(telemetryPort,telemetry)]);
  const scene={polygons:[],surfaceZones:[],chargingStations:[],motion:{cruiseSpeedMps:0.22,payloadKg:0,batteryRange:100}};
  const response=await fetch(`http://127.0.0.1:${planningPort}/api/solve-route`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({points:[{x:0,y:0},{x:1,y:0}],scene,task:"tsp"})});
  const plan=await response.json();assert.equal(plan.ok,true,JSON.stringify(plan));assert.ok(plan.planning.routeEnergy>0);
  socket=new WebSocket(`ws://127.0.0.1:${routePort}/ui`);await once(socket,"open");
  const command=createRouteCommand({source:"services-smoke",requestId:"smoke-mission",payload:{type:"route",route:plan.route,seedRoute:plan.seedRoute,scene}});
  const ack=await send(command);assert.equal(ack.ok,true,JSON.stringify(ack));assert.equal(ack.missionId,"smoke-mission");
  const csv=await readFile(path.join(directory,"route.csv"),"utf8");assert.match(csv,/# command smoke-mission/);
  assert.equal((await send(command)).ok,true);assert.equal(await readFile(path.join(directory,"route.csv"),"utf8"),csv);
  // Synthetic controller feedback: this is a transport test, not a physics simulation.
  await writeFile(path.join(directory,"robot_state.json"),JSON.stringify({navigation:{missionId:"another-mission",finished:true}}));
  assert.equal((await getMission()).status,"persisted");
  await writeFile(path.join(directory,"robot_state.json"),JSON.stringify({navigation:{missionId:"smoke-mission",finished:true}}));
  assert.equal((await getMission()).status,"completed");
  socket.terminate();socket=null;await stop(missions);missions=start("route");await waitReady(routePort,missions);
  assert.equal((await getMission()).status,"completed");
  socket=new WebSocket(`ws://127.0.0.1:${routePort}/ui`);await once(socket,"open");
  assert.equal((await send(command)).status,"completed");assert.equal(await readFile(path.join(directory,"route.csv"),"utf8"),csv);
  console.log("services smoke passed: independent processes, full planning, mission ACK, idempotency, matched feedback, restart");
} finally {
  socket?.terminate();await Promise.all(children.map(stop));
  await rm(directory,{recursive:true,force:true});
}
