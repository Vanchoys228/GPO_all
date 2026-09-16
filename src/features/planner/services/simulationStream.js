// Webots R2025a MJPEG handshake; execution commands stay in the mission service.
export function connectSimulationStream({url,onFrame,onStatus,createSocket=value=>new WebSocket(value)}) {
  let stopped=false, socket, retry, watchdog;
  const httpBase=new URL(url);
  httpBase.protocol=httpBase.protocol === "wss:" ? "https:" : "http:";
  const connect=() => {
    if(stopped)return;
    onStatus("connecting");
    socket=createSocket(url);
    watchdog=setTimeout(()=>socket.close(),15000);
    socket.onopen=()=>socket.send("mjpeg: 960x540");
    socket.onmessage=event=>{
      if(stopped || typeof event.data !== "string")return;
      if(event.data.startsWith("multimedia: ")) {
        const frame=new URL(event.data.split(" ")[1],httpBase);
        if(frame.origin!==httpBase.origin || !frame.pathname.startsWith(httpBase.pathname))return;
        clearTimeout(watchdog);
        onFrame(frame.href);
        onStatus("connected");
      }
      if(event.data === "scene load completed")socket.send("resize: 960x540");
    };
    socket.onerror=()=>socket.close();
    socket.onclose=()=>{
      clearTimeout(watchdog);
      if(stopped)return;
      onFrame(null);
      onStatus("reconnecting");
      retry=setTimeout(connect,2000);
    };
  };
  connect();
  const dispose=()=>{
    stopped=true;
    clearTimeout(retry);clearTimeout(watchdog);
    socket.onclose=null;socket.onmessage=null;socket.onerror=null;socket.onopen=null;
    socket.close();
  };
  dispose.reconnect=()=>{if(!stopped)socket.close();};
  return dispose;
}
