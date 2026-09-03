import { ROUTE_WS_URL } from "../../../lib/runtimeConfig";
import { sanitizeRouteForController } from "../../../lib/zonePlanner";
import {
  buildRouteEnergyStats,
  buildRouteWithEnergyStops,
  getEnergyWarningText,
} from "../model/routeEnergy";
import {
  buildControllerRoutePayload,
  buildRouteCommand,
  buildRouteSentStatus,
} from "../model/routeCommand";

export const usePlannerRouteSender = ({
  algorithmKey,
  batteryRangeMeters,
  cruiseSpeedMps,
  energyOptions,
  optimizedRoute,
  payloadKg,
  plannerModel,
  routeSeed,
  routeSocketRef,
  routeTaskKey,
  selectedAlgorithmParams,
  setEnergyWarning,
  setRouteEnergyStats,
  setStatus,
  startRouteTiming,
}) => {
  const sendRoute = () => {
    if (!optimizedRoute.length) {
      setStatus("Сначала постройте маршрут.");
      return;
    }
    if (plannerModel.routeBlocked) {
      setStatus("Маршрут всё ещё пересекает ограничивающий контур.");
      return;
    }

    let controllerRouteSource = optimizedRoute;
    let chargingStops = 0;
    if (routeSeed.length > 1) {
      const rebuilt = buildRouteWithEnergyStops({
        seedRoute: routeSeed,
        polygons: plannerModel.polygons,
        surfaceZones: plannerModel.surfaceZones,
        chargingStations: plannerModel.chargePoints,
        batteryRangeMeters,
        energyOptions,
      });
      if (!rebuilt.ok) {
        setEnergyWarning(getEnergyWarningText(rebuilt));
        setStatus(
          rebuilt.error || "Невозможно безопасно построить маршрут через текущие зоны."
        );
        return;
      }
      controllerRouteSource = rebuilt.route;
      chargingStops = rebuilt.stationStopCount;
      setRouteEnergyStats(buildRouteEnergyStats(rebuilt));
    }

    const routeForController = sanitizeRouteForController(controllerRouteSource);
    if (routeForController.length < 2) {
      setStatus("Маршрут слишком короткий после очистки.");
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
    const command = buildRouteCommand(payload);
    const sendPayload = (socket) => {
      socket.send(JSON.stringify(command));
      startRouteTiming();
      setEnergyWarning("");
      setStatus(buildRouteSentStatus(routeForController.length, chargingStops));
    };

    const socket = routeSocketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      const temporarySocket = new WebSocket(ROUTE_WS_URL);
      routeSocketRef.current = temporarySocket;
      temporarySocket.onopen = () => sendPayload(temporarySocket);
      temporarySocket.onclose = () => {
        if (routeSocketRef.current === temporarySocket) routeSocketRef.current = null;
      };
      temporarySocket.onerror = () => setStatus("Ошибка соединения с маршрутом.");
      return;
    }
    sendPayload(socket);
  };

  return sendRoute;
};
