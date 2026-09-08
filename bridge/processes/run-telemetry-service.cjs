const {startTelemetryProcess} = require("./telemetry-process.cjs");
const {installShutdown} = require("./shutdown.cjs");
const server = startTelemetryProcess();
installShutdown(() => server.close());
