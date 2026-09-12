require("../operations/logger.cjs").installLogging("gateway");
const {startGatewayProcess}=require("./gateway-process.cjs");
const {installShutdown}=require("./shutdown.cjs");
const server=startGatewayProcess();
installShutdown(()=>server.close());
