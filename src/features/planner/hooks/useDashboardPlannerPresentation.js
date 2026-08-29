import { useMemo } from "react";
import { analyzeRouteInfluence } from "../../../lib/energyModel";

export const useDashboardPlannerPresentation = ({
  optimizedRoute,
  plannerModel,
  payloadKg,
  cruiseSpeedMps,
  routeEnergyStats,
  routeTimingDisplay,
  routeAvoidanceTimeSec,
  routeOffRouteActive,
  telemetry,
}) => {
  const telemetryForSidebar = useMemo(
    () => ({
      ...telemetry,
      navigation: {
        ...telemetry.navigation,
        avoidanceTimeSec: routeAvoidanceTimeSec,
        offRouteActive: routeOffRouteActive,
      },
    }),
    [routeAvoidanceTimeSec, routeOffRouteActive, telemetry]
  );
  const routeInfluenceRows = useMemo(
    () => analyzeRouteInfluence(optimizedRoute, {
      surfaceZones: plannerModel.surfaceZones,
      speedMps: cruiseSpeedMps,
      payloadKg,
      stationStopCount: routeEnergyStats.stationStopCount,
      plannedTimeSec: routeEnergyStats.estimatedTimeSec,
      actualTimeSec: routeTimingDisplay.actualTimeSec,
      avoidanceTimeSec: routeAvoidanceTimeSec,
    }),
    [cruiseSpeedMps, optimizedRoute, payloadKg, plannerModel.surfaceZones,
      routeAvoidanceTimeSec, routeEnergyStats.estimatedTimeSec,
      routeEnergyStats.stationStopCount, routeTimingDisplay.actualTimeSec]
  );

  return { routeInfluenceRows, telemetryForSidebar };
};
