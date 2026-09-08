const installShutdown = (close) => {
  let stopping = false;
  const stop = async () => {
    if (stopping) return;
    stopping = true;
    try {await close();process.exitCode=0;}
    catch(error) {console.error("[service] shutdown failed:",error.message);process.exitCode=1;}
  };
  process.once("SIGINT",stop);
  process.once("SIGTERM",stop);
};
module.exports = {installShutdown};
