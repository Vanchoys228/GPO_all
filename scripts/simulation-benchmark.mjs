// Measures simulation time and W3D updates (not browser render FPS).
// Open the dashboard viewer as well to include client-side rendering in the workload.
import WebSocket from "ws";
import {connectSimulationStream} from "../src/features/planner/services/simulationStream.js";

const seconds=Number(process.env.BENCHMARK_SECONDS || 20);
if (!Number.isFinite(seconds) || seconds < 5 || seconds > 300) throw new Error("BENCHMARK_SECONDS must be 5..300");
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let latest, observed=0, connected=false, camera=false, updates=0, bytes=0;
const telemetry=new WebSocket(process.env.TELEMETRY_WS || "ws://127.0.0.1:9001");
telemetry.on("error",error=>console.error(error.message));
telemetry.on("message",data=>{
  const event=JSON.parse(data);
  if(event.type === "telemetry.event") {
    latest=event.payload.simulationTime;
    camera=Boolean(event.payload.perception?.camera?.frameDataUrl);
    observed=performance.now();
  }
});
const connection=connectSimulationStream({
  url:process.env.SIMULATION_WS || "ws://127.0.0.1:8080/simulation/",
  createSocket:url=>new WebSocket(url),
  onUpdate:value=>{updates++;bytes+=Buffer.byteLength(value);},onStatus:value=>{connected=value === "connected";},
});
try {
  const deadline=performance.now()+30000;
  while(!connected || !Number.isFinite(latest) || !camera) {
    if(performance.now()>deadline) throw new Error("Simulator/camera did not become ready");
    await delay(100);
  }
  for(const mode of ["realtime","fast"]) {
    await connection.setMode(mode);
    await delay(3000);
    const start=performance.now(), simStart=latest, bytesStart=bytes, updatesStart=updates;
    await delay(seconds*1000);
    if (!connected || performance.now()-observed>3000) throw new Error("Telemetry/stream lost during benchmark");
    if (updates===updatesStart) throw new Error("W3D updates stopped during benchmark");
    console.log(JSON.stringify({mode,wallSeconds:(performance.now()-start)/1000,simulationSeconds:latest-simStart,speed:Number(((latest-simStart)/((performance.now()-start)/1000)).toFixed(3)),sceneUpdatesPerSecond:(updates-updatesStart)/seconds,streamBytes:bytes-bytesStart}));
  }
} finally {
  // Leave the simulator in the ordinary mode after this benchmark.
  if(connected) await connection.setMode("realtime").catch(()=>{});
  connection();telemetry.terminate();
}
