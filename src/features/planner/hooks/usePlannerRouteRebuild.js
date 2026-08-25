import { useEffect, useRef } from "react";
import { sanitizeRouteForController } from "../../../lib/zonePlanner";
import { sendRouteChannelPayload } from "../services/routeChannel";
import {
  buildRouteEnergyStats,
  buildRouteWithEnergyStops,
  createEmptyRouteEnergyStats,
  getEnergyWarningText,
} from "../model/routeEnergy";
import {
  buildAutoRouteUpdatedStatus,
  buildControllerRoutePayload,
} from "../model/routeCommand";

export const usePlannerRouteRebuild = ({
  algorithmKey,
  batteryRangeMeters,
  chargePointsRoutingText,
  cruiseSpeedMps,
  energyOptions,
  payloadKg,
  previewPolygonRoutingText,
  routeSeed,
  routeSocketRef,
  routeTaskKey,
  selectedAlgorithmParams,
  setEnergyWarning,
  setOptimizedRoute,
  setRouteEnergyStats,
  setStatus,
  startRouteTiming,
  surfaceSyncPayloadText,
  surfaceZones,
  zoneSyncPayloadText,
}) => {
  const lastAutoRouteZoneSyncRef = useRef(null);
  const autoRouteSyncToken = `${zoneSyncPayloadText}|${chargePointsRoutingText}|${surfaceSyncPayloadText}|${batteryRangeMeters}`;

  useEffect(() => {
    if (!routeSeed.length) {
      setOptimizedRoute([]);
      setEnergyWarning("");
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      return;
    }
    const nextRoute = buildRouteWithEnergyStops({
      seedRoute: routeSeed,
      polygons: JSON.parse(previewPolygonRoutingText),
      surfaceZones,
      chargingStations: JSON.parse(chargePointsRoutingText),
      batteryRangeMeters,
      energyOptions,
    });
    if (!nextRoute.ok) {
      setOptimizedRoute([]);
      setEnergyWarning(getEnergyWarningText(nextRoute));
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      setStatus(nextRoute.error || "Маршрут недостижим при текущих ограничениях.");
      return;
    }
    setEnergyWarning("");
    setOptimizedRoute(nextRoute.route);
    setRouteEnergyStats(buildRouteEnergyStats(nextRoute));
  }, [
    batteryRangeMeters,
    chargePointsRoutingText,
    energyOptions,
    previewPolygonRoutingText,
    routeSeed,
    setEnergyWarning,
    setOptimizedRoute,
    setRouteEnergyStats,
    setStatus,
    surfaceZones,
  ]);

  useEffect(() => {
    if (lastAutoRouteZoneSyncRef.current === autoRouteSyncToken) return;
    lastAutoRouteZoneSyncRef.current = autoRouteSyncToken;
    if (routeSeed.length < 2) {
      setEnergyWarning("");
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      return;
    }
    const controllerPolygonsPayload = JSON.parse(zoneSyncPayloadText);
    const controllerPolygons = (controllerPolygonsPayload?.zones || []).map((zone) => ({
      id: zone.id,
      name: zone.name,
      points: Array.isArray(zone.points) ? zone.points : [],
    }));
    const rebuilt = buildRouteWithEnergyStops({
      seedRoute: routeSeed,
      polygons: controllerPolygons,
      surfaceZones,
      chargingStations: JSON.parse(chargePointsRoutingText),
      batteryRangeMeters,
      energyOptions,
    });
    if (!rebuilt.ok) {
      setEnergyWarning(getEnergyWarningText(rebuilt));
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      setStatus(rebuilt.error || "Невозможно безопасно перестроить маршрут.");
      return;
    }
    setEnergyWarning("");
    setRouteEnergyStats(buildRouteEnergyStats(rebuilt));
    const routeForController = sanitizeRouteForController(rebuilt.route);
    if (routeForController.length < 2) {
      setStatus("Маршрут стал слишком коротким после перестройки под зоны.");
      return;
    }
    const payload = buildControllerRoutePayload({
      algorithmKey,
      batteryRangeMeters,
      cruiseSpeedMps,
      payloadKg,
      route: routeForController,
      routeTaskKey,
      selectedAlgorithmParams,
    });
    sendRouteChannelPayload(routeSocketRef, payload, {
      onSent: () => {
        startRouteTiming();
        setStatus(
          buildAutoRouteUpdatedStatus(routeForController.length, rebuilt.stationStopCount)
        );
      },
    });
  }, [
    algorithmKey,
    autoRouteSyncToken,
    batteryRangeMeters,
    chargePointsRoutingText,
    cruiseSpeedMps,
    energyOptions,
    payloadKg,
    routeSeed,
    routeSocketRef,
    routeTaskKey,
    selectedAlgorithmParams,
    setEnergyWarning,
    setRouteEnergyStats,
    setStatus,
    startRouteTiming,
    surfaceZones,
    zoneSyncPayloadText,
  ]);
};
