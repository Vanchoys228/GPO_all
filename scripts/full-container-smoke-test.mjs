import {spawn} from "node:child_process";
import {once} from "node:events";
import assert from "node:assert/strict";
import WebSocket from "ws";
import {createRouteCommand} from "../shared/contracts/index.js";

const project=`gpo-full-test-${Date.now()}`;
const root=new URL("../",import.meta.url);
const sockets=[];
const compose=(...args)=>new Promise((resolve,reject)=>{
  const child=spawn("docker",["compose","-p",project,"-f","compose.yaml","-f","compose.simulator.yaml",...args],{cwd:root,windowsHide:true,stdio:"inherit"});
  child.on("error",reject);child.on("exit",code=>code===0 ? resolve() : reject(new Error(`Compose exit ${code}`)));
});
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function until(condition,label,timeout=120000) {
  const deadline=Date.now()+timeout;
  while(Date.now()<deadline){if(await condition())return;await delay(250);}
  throw new Error(`Timeout: ${label}`);
}
const mission=async id=>(await fetch(`http://127.0.0.1:9002/api/missions/${id}`)).json();
async function connect(url) {
  const socket=new WebSocket(url);sockets.push(socket);await once(socket,"open");return socket;
}
async function send(socket,id,route) {
  const command=createRouteCommand({source:"full-container-test",requestId:id,payload:{type:"route",route,scene:{polygons:[],surfaceZones:[],chargingStations:[],motion:{cruiseSpeedMps:0.22,payloadKg:0,batteryRange:100}}}});
  const reply=new Promise((resolve,reject)=>{
    const timer=setTimeout(()=>{socket.off("message",receive);reject(new Error("ACK timeout"));},15000);
    function receive(data){const ack=JSON.parse(data);if(ack.requestId===id){clearTimeout(timer);socket.off("message",receive);resolve(ack);}}
    socket.on("message",receive);
  });
  socket.send(JSON.stringify(command));assert.equal((await reply).ok,true);
}
try {
  await compose("up","-d","--no-build","--wait","--wait-timeout","180");
  const plan=await (await fetch("http://127.0.0.1:9003/api/solve-route",{
    method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({points:[{x:0,y:0},{x:1,y:0}],task:"tsp"}),
  })).json();
  assert.equal(plan.ok,true,JSON.stringify(plan));
  let telemetry;
  const telemetrySocket=await connect("ws://127.0.0.1:9001");
  telemetrySocket.on("message",data=>{const event=JSON.parse(data);if(event.type==="telemetry.event")telemetry=event.payload;});
  await until(()=>telemetry?.perception?.camera?.frameDataUrl,"robot camera");
  assert.equal(telemetry.navigation.status,"waiting_for_route","fresh stack must not move before a mission");
  const streaming=await connect("ws://127.0.0.1:8080/simulation/");
  let imageUrl;
  streaming.on("message",data=>{
    const message=data.toString();
    if(message.startsWith("multimedia: "))imageUrl=new URL(message.split(" ")[1],"http://127.0.0.1:8080/simulation/");
    if(message==="scene load completed")streaming.send("resize: 960x540");
  });
  streaming.send("mjpeg: 960x540");
  await until(()=>imageUrl,"streaming handshake",30000);
  const response=await fetch(imageUrl,{signal:AbortSignal.timeout(20000)});
  assert.equal(response.status,200);
  assert.match(response.headers.get("content-type"),/multipart\/x-mixed-replace/);
  const reader=response.body.getReader();let bytes=Buffer.alloc(0);
  while(bytes.indexOf(Buffer.from([0xff,0xd9]))<0){const part=await reader.read();assert.ok(!part.done);bytes=Buffer.concat([bytes,Buffer.from(part.value)]);}
  await reader.cancel();assert.ok(bytes.indexOf(Buffer.from([0xff,0xd8]))>=0,"stream must contain a JPEG frame");
  const socket=await connect("ws://127.0.0.1:9002/ui");
  await send(socket,"complete",[{x:0,y:0},{x:1,y:0}]);
  await until(async()=>(await mission("complete")).status==="completed","physical route completion");
  await compose("kill","-s","SIGKILL","route");
  await compose("up","-d","--wait","route");
  assert.equal((await mission("complete")).status,"completed");
  const nextSocket=await connect("ws://127.0.0.1:9002/ui");
  await compose("stop","simulator");
  await send(nextSocket,"restart-cancel",[{x:0,y:0},{x:2,y:0}]);
  await fetch("http://127.0.0.1:9002/api/missions/restart-cancel/cancel",{method:"POST"});
  await compose("up","-d","--wait","simulator");
  await until(async()=>(await mission("restart-cancel")).status==="cancelled","cancel after simulator restart");
  await send(nextSocket,"after-restart",[{x:0,y:0},{x:1,y:0}]);
  await until(async()=>(await mission("after-restart")).status==="completed","route after simulator restart");
  console.log("PASS: six containers, camera, MJPEG through nginx, physical missions, route crash and simulator cancellation recovery");
} finally {
  for(const socket of sockets)socket.terminate();
  // The unique project above is owned solely by this test; its volumes are disposable.
  await compose("down","-v","--timeout","15");
}
