export const buildControllerRoutePayload = ({
  algorithmKey,
  batteryRangeMeters,
  cruiseSpeedMps,
  payloadKg,
  route,
  routeTaskKey,
  selectedAlgorithmParams,
}) => ({
  type: "route",
  algorithm: {
    key: algorithmKey,
    task: routeTaskKey,
    params: selectedAlgorithmParams,
  },
  motion: {
    cruiseSpeedMps,
    payloadKg,
    batteryRange: batteryRangeMeters,
  },
  route: route.map((point) => ({ x: point.x, y: point.y })),
});

export const buildRouteCommand = (
  payload,
  { requestId = crypto.randomUUID(), timestamp } = {}
) => createRouteCommand({
  source: "frontend",
  requestId,
  timestamp,
  payload,
});

export const buildRouteSentStatus = (pointCount, chargingStops) => {
  const chargingSuffix = chargingStops ? `, зарядок: ${chargingStops}` : "";
  return `Маршрут отправлен (${pointCount} точек${chargingSuffix}).`;
};

export const buildAutoRouteUpdatedStatus = (pointCount, chargingStops) => {
  const chargingSuffix = chargingStops ? `, зарядок: ${chargingStops}` : "";
  return `Маршрут обновлён после изменения ограничивающих зон (${pointCount} точек${chargingSuffix}).`;
};
import { createRouteCommand } from "../../../../shared/contracts/index.js";
