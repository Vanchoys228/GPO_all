import { sendRouteChannelPayload } from "../services/routeChannel";
import { buildSceneSnapshot } from "../model/sceneSnapshot";
import { buildControllerRoutePayload, buildRouteCommand, buildRouteSentStatus } from "../model/routeCommand";

export const usePlannerRouteSender = ({algorithmKey,batteryRangeMeters,cruiseSpeedMps,energyOptions,
  optimizedRoute,payloadKg,plannerModel,routeSeed,routeSocketRef,routeTaskKey,selectedAlgorithmParams,
  setEnergyWarning,setStatus,onMissionSubmitted}) => {
  return () => {
    if (!optimizedRoute.length) { setStatus("Сначала постройте маршрут."); return; }
    if (plannerModel.routeBlocked) { setStatus("Маршрут всё ещё пересекает ограничивающий контур."); return; }
    const payload = buildControllerRoutePayload({algorithmKey,batteryRangeMeters,cruiseSpeedMps,payloadKg,
      route:optimizedRoute,routeTaskKey,selectedAlgorithmParams});
    payload.seedRoute = routeSeed.length > 1 ? routeSeed : optimizedRoute;
    payload.scene = buildSceneSnapshot({plannerModel,batteryRangeMeters,energyOptions});
    setStatus("Сервис проверяет и сохраняет маршрут...");
    return sendRouteChannelPayload(routeSocketRef,buildRouteCommand(payload),{
      timeoutMs:35000,
      onSent: acknowledgement => {
        setEnergyWarning("");
        setStatus(buildRouteSentStatus(acknowledgement.route?.length || optimizedRoute.length,acknowledgement.planning?.stationStopCount || 0));
        onMissionSubmitted?.(acknowledgement);
      },
      onError:error => setStatus(error.message),
    });
  };
};
