import { useMemo } from "react";
import { buildPlannerModel } from "../../../lib/plannerModel";
import { buildPlannerSyncPayloads } from "../model/plannerSync";
import { createDashboardPlannerViewModel } from "../model/dashboardPlannerViewModel";
import { useDashboardPlannerPresentation } from "./useDashboardPlannerPresentation";
import { usePlannerBridgeSync } from "./usePlannerBridgeSync";

export const useDashboardPlannerDerivedState = (state, runtime) => {
  const { algorithm, energy, limits, route, surfaces } = state;
  const plannerModel = buildPlannerModel({
    points: route.points,
    limitZones: limits.limitZones,
    optimizedRoute: route.optimizedRoute,
    activeLimitZoneId: limits.activeLimitZoneId,
    surfaceZones: surfaces.surfaceZones,
  });
  const viewModel = createDashboardPlannerViewModel({
    algorithmKey: algorithm.algorithmKey,
    algorithmParams: algorithm.algorithmParams,
    activeSurfaceZoneId: surfaces.activeSurfaceZoneId,
    surfaceZones: surfaces.surfaceZones,
  });
  const syncPayloads = buildPlannerSyncPayloads(plannerModel);

  usePlannerBridgeSync({
    batteryRangeMeters: energy.batteryRangeMeters,
    cruiseSpeedMps: energy.cruiseSpeedMps,
    payloadKg: energy.payloadKg,
    routeSocketRef: runtime.routeWsRef,
    surfaceSyncPayloadText: syncPayloads.surfaceSyncPayloadText,
    zoneSyncPayloadText: syncPayloads.zoneSyncPayloadText,
  });

  const energyOptions = useMemo(
    () => ({ speedMps: energy.cruiseSpeedMps, payloadKg: energy.payloadKg }),
    [energy.cruiseSpeedMps, energy.payloadKg]
  );
  const presentation = useDashboardPlannerPresentation({
    optimizedRoute: route.optimizedRoute,
    plannerModel,
    payloadKg: energy.payloadKg,
    cruiseSpeedMps: energy.cruiseSpeedMps,
    routeEnergyStats: energy.routeEnergyStats,
    routeTimingDisplay: runtime.routeTiming.display,
    routeAvoidanceTimeSec: runtime.routeTiming.avoidanceTimeSec,
    routeOffRouteActive: runtime.routeTiming.offRouteActive,
    telemetry: runtime.telemetry,
  });

  return { plannerModel, viewModel, syncPayloads, energyOptions, ...presentation };
};
