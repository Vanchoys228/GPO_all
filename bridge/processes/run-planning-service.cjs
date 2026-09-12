require("../operations/logger.cjs").installLogging("planning");
const {startPlanningProcess} = require("./planning-process.cjs");
const {installShutdown} = require("./shutdown.cjs");
const server = startPlanningProcess();
installShutdown(() => new Promise(resolve => server.close(resolve)));
