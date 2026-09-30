require("../operations/logger.cjs").installLogging("route");
const {startRouteProcess} = require("./route-process.cjs");
const {installShutdown} = require("./shutdown.cjs");
const server = startRouteProcess();
installShutdown(() => server.close());
