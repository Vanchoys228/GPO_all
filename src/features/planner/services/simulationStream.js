// Headless W3D protocol client for diagnostics; browser rendering uses simulationViewer.
export function connectSimulationStream({url,onScene=()=>{},onUpdate=()=>{},onStatus,onMode=()=>{},createSocket=value=>new WebSocket(value)}) {
  let stopped=false, socket, retry, watchdog, connected=false, pending=null;
  const rejectPending=message=>{
    if (!pending) return;
    clearTimeout(pending.timer);
    pending.reject(new Error(message));
    pending=null;
  };
  const receiveMode=value=>{
    const mode=value === "real-time" ? "realtime" : value === "fast" ? "fast" : value.startsWith("pause") ? "paused" : null;
    if (!mode) return;
    onMode(mode);
    if (pending?.mode === mode) {
      clearTimeout(pending.timer);
      pending.resolve(mode);
      pending=null;
    }
  };
  const connect=() => {
    if(stopped)return;
    onStatus("connecting");
    socket=createSocket(url);
    watchdog=setTimeout(()=>socket.close(),60000);
    socket.onopen=()=>socket.send("w3d;broadcast");
    socket.onmessage=event=>{
      if(stopped || typeof event.data !== "string")return;
      if(event.data === "scene load completed") {
        clearTimeout(watchdog);
        connected=true;
        onStatus("connected");
      }
      if(event.data.startsWith("model:"))onScene(event.data.slice(6));
      if(event.data.startsWith("application/json:"))onUpdate(event.data.slice(17));
      receiveMode(event.data);
    };
    socket.onerror=()=>socket.close();
    socket.onclose=()=>{
      clearTimeout(watchdog);
      connected=false;
      rejectPending("Соединение с симулятором потеряно.");
      if(stopped)return;
      onMode(null);
      onStatus("reconnecting");
      retry=setTimeout(connect,2000);
    };
  };
  connect();
  const dispose=()=>{
    stopped=true;
    connected=false;
    rejectPending("Соединение с симулятором закрыто.");
    clearTimeout(retry);clearTimeout(watchdog);
    socket.onclose=null;socket.onmessage=null;socket.onerror=null;socket.onopen=null;
    socket.close();
  };
  dispose.reconnect=()=>{if(!stopped)socket.close();};
  dispose.setMode=mode=>{
    if (!["realtime","fast"].includes(mode)) return Promise.reject(new Error("Неизвестный режим симуляции."));
    if (stopped || !connected || socket.readyState !== 1) return Promise.reject(new Error("Симулятор не подключён."));
    if (pending) return Promise.reject(new Error("Дождитесь подтверждения режима."));
    return new Promise((resolve,reject)=>{
      pending={mode,resolve,reject,timer:setTimeout(()=>rejectPending("Симулятор не подтвердил изменение скорости."),5000)};
      try {socket.send(mode === "fast" ? "fast:-1" : "real-time:-1");}
      catch {rejectPending("Не удалось отправить режим симулятору.");}
    });
  };
  return dispose;
}
