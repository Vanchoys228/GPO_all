import {
  getAlgorithmLabel,
  getTaskLabel,
  solveRouteWithNativeAlgorithm,
} from "../../../lib/routeAlgorithms";
import {
  getRouteAnchor,
  rotateClosedRouteToNearestPoint,
} from "../../../lib/plannerModel";
import { routeCrossesAnyLimitPolygon } from "../../../lib/zonePlanner";
import {
  buildRouteEnergyStats,
  buildRouteWithEnergyStops,
  createEmptyRouteEnergyStats,
  getEnergyWarningText,
} from "../model/routeEnergy";
import { buildRouteOptimizationStatus } from "../model/routeOptimization";

export const usePlannerRouteOptimization = ({ algorithmKey, batteryRangeMeters, energyOptions, isOptimizing, plannerModel, routeTaskKey, selectedAlgorithmParams, setEnergyWarning, setIsOptimizing, setOptimizedRoute, setRouteEnergyStats, setRouteSeed, setStatus, telemetry }) => {
  const optimizeRoute = async () => {
    if (isOptimizing) return;
    if (plannerModel.visitPoints.length < 2) {
      setStatus("Добавьте хотя бы две точки посещения.");
      setEnergyWarning("");
      return;
    }
    setIsOptimizing(true);
    setStatus("Строим маршрут...");
    try {
      const solveResult = await solveRouteWithNativeAlgorithm(plannerModel.visitPoints, algorithmKey, selectedAlgorithmParams, routeTaskKey);
      let solvedRoute = solveResult.route;
      if (routeTaskKey === "tsp" && solvedRoute.length) {
        solvedRoute = rotateClosedRouteToNearestPoint(solvedRoute, getRouteAnchor(telemetry));
      }
      const routed = buildRouteWithEnergyStops({ seedRoute: solvedRoute, polygons: plannerModel.previewPolygons, surfaceZones: plannerModel.surfaceZones, chargingStations: plannerModel.chargePoints, batteryRangeMeters, energyOptions });
      if (!routed.ok) {
        setRouteSeed(solvedRoute);
        setOptimizedRoute([]);
        setRouteEnergyStats(createEmptyRouteEnergyStats());
        setEnergyWarning(getEnergyWarningText(routed));
        setStatus(routed.error || "Не удалось построить достижимый маршрут.");
        return;
      }
      setRouteSeed(solvedRoute);
      setOptimizedRoute(routed.route);
      setRouteEnergyStats(buildRouteEnergyStats(routed));
      setEnergyWarning("");
      setStatus(buildRouteOptimizationStatus({ adjustedVisitCount: plannerModel.adjustedVisits.length, algorithmLabel: getAlgorithmLabel(algorithmKey), blocked: routeCrossesAnyLimitPolygon(routed.route, plannerModel.previewPolygons), routed, taskLabel: getTaskLabel(routeTaskKey) }));
    } catch (error) {
      setRouteSeed([]);
      setOptimizedRoute([]);
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      setEnergyWarning("");
      setStatus(error instanceof Error ? error.message : "Не удалось построить маршрут.");
    } finally {
      setIsOptimizing(false);
    }
  };
  return optimizeRoute;
};
