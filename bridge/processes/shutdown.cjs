const installShutdown = (close, { runtime = process, timeoutMs = 10000 } = {}) => {
  let stopping = false;
  let timer;
  const dispose = () => {
    clearTimeout(timer);
    runtime.removeListener("SIGINT", stop);
    runtime.removeListener("SIGTERM", stop);
  };
  const stop = async () => {
    if (stopping) return;
    stopping = true;
    timer = setTimeout(() => {
      console.error("[service] shutdown deadline exceeded");
      runtime.exit(1);
    }, timeoutMs);
    timer.unref();
    try {await close();runtime.exitCode ||= 0;}
    catch(error) {console.error("[service] shutdown failed:",error.message);runtime.exitCode=1;}
    finally { dispose(); }
  };
  runtime.on("SIGINT",stop);
  runtime.on("SIGTERM",stop);
  return dispose;
};
module.exports = {installShutdown};
