const CHANNEL="gpo-w3d";
const modes=new Set(["realtime","fast","paused"]);

// The official viewer is isolated in an iframe because it owns global WebGL state.
export function connectSimulationViewer({frame,onStatus,onMode,onError=()=>{},host=window,newSession=()=>crypto.randomUUID()}) {
  let stopped=false,connected=false,session,retry,watchdog,pending;
  const rejectPending=message=>{
    if(!pending)return;
    clearTimeout(pending.timer);pending.reject(new Error(message));pending=null;
  };
  const reconnect=()=>{
    if(stopped || retry)return;
    connected=false;clearTimeout(watchdog);
    rejectPending("Соединение с симулятором потеряно.");
    onMode(null);onStatus("reconnecting");
    retry=setTimeout(()=>{retry=null;start();},2000);
  };
  const start=()=>{
    session=newSession();connected=false;onStatus("connecting");
    watchdog=setTimeout(reconnect,60000);
    // A query change creates a new document; a hash change would keep the dead socket.
    frame.src=`/webots/viewer.html?session=${encodeURIComponent(session)}`;
  };
  const receive=event=>{
    const message=event.data;
    if(stopped || event.origin !== host.location.origin || event.source !== frame.contentWindow ||
       message?.channel !== CHANNEL || message.session !== session)return;
    if(message.type === "ready") {
      connected=true;clearTimeout(watchdog);onStatus("connected");onError("");
    } else if(message.type === "mode" && modes.has(message.mode)) {
      onMode(message.mode);
      if(pending?.mode === message.mode) {
        clearTimeout(pending.timer);pending.resolve(message.mode);pending=null;
      }
    } else if(message.type === "disconnected")reconnect();
    else if(message.type === "error") {
      const reason=typeof message.message === "string" ? message.message : "Ошибка просмотрщика Webots.";
      rejectPending(reason);onError(reason);
      if(message.fatal) {
        clearTimeout(watchdog);clearTimeout(retry);retry=null;connected=false;onMode(null);onStatus("error");
      }
    }
  };
  host.addEventListener("message",receive);start();
  const dispose=()=>{
    stopped=true;connected=false;clearTimeout(watchdog);clearTimeout(retry);
    rejectPending("Просмотрщик закрыт.");host.removeEventListener("message",receive);
  };
  dispose.setMode=mode=>{
    if(!["realtime","fast"].includes(mode))return Promise.reject(new Error("Неизвестный режим симуляции."));
    if(stopped || !connected)return Promise.reject(new Error("Симулятор не подключён."));
    if(pending)return Promise.reject(new Error("Дождитесь подтверждения режима."));
    return new Promise((resolve,reject)=>{
      pending={mode,resolve,reject,timer:setTimeout(()=>rejectPending("Симулятор не подтвердил изменение скорости."),5000)};
      frame.contentWindow.postMessage({channel:CHANNEL,session,type:"set-mode",mode},host.location.origin);
    });
  };
  return dispose;
}
