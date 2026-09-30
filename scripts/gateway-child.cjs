require("../bridge/operations/logger.cjs").installLogging("gateway");
const { startGatewayProcess } = require("../bridge/processes/gateway-process.cjs");
const { installShutdown } = require("../bridge/processes/shutdown.cjs");
const gateway = startGatewayProcess();
installShutdown(async () => { await gateway.close(); if (process.connected) process.disconnect(); });
process.on("message", message => { if (message === "shutdown") process.emit("SIGTERM"); });
process.on("disconnect", () => process.emit("SIGTERM"));
