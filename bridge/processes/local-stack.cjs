const { startPlanningProcess: defaultStartPlanningProcess } = require("./planning-process.cjs");
const { startRouteProcess: defaultStartRouteProcess } = require("./route-process.cjs");
const { startTelemetryProcess: defaultStartTelemetryProcess } = require("./telemetry-process.cjs");

const startLocalStack = ({
  enableMockTelemetry = process.env.MOCK_TELEMETRY === "1",
  startPlanningProcess = defaultStartPlanningProcess,
  startRouteProcess = defaultStartRouteProcess,
  startTelemetryProcess = defaultStartTelemetryProcess,
} = {}) => {
  const telemetryServer = startTelemetryProcess({ enableMockTelemetry });
  const routeServer = startRouteProcess();
  const planningServer = startPlanningProcess();

  return {
    async stop() {
      await Promise.allSettled([
        telemetryServer.close(),
        routeServer.close(),
        new Promise((resolve) => planningServer.close(resolve)),
      ]);
    },
  };
};

module.exports = { startLocalStack };
