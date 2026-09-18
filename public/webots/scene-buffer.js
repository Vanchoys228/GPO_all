// W3D updates are deltas: dropping even the initial pose while parsing the model
// can leave a stationary robot at its old position. Keep order until nodes exist.
export function bufferSceneUpdates(socket) {
  const receive=socket.onmessage;
  let loading=true,queue=[];
  socket.onmessage=event=>{
    const data=event.data;
    if(typeof data === "string" && data.startsWith("model:")) {
      loading=true;queue=[];
    }
    if(loading && typeof data === "string" &&
       ["application/json:","node:","delete:"].some(prefix=>data.startsWith(prefix))) {
      queue.push(event);return;
    }
    receive.call(socket,event);
  };
  return ()=>{
    loading=false;
    const buffered=queue;queue=[];
    for(const event of buffered)receive.call(socket,event);
  };
}
