import { useEffect, useRef } from "react";
import { sendRouteChannelPayload } from "../services/routeChannel";

export const usePlannerBridgeSync = ({
  batteryRangeMeters,
  cruiseSpeedMps,
  payloadKg,
  routeSocketRef,
  surfaceSyncPayloadText,
  zoneSyncPayloadText,
}) => {
  const motionProfileTouchedRef = useRef(false);

  useEffect(() => {
    sendRouteChannelPayload(routeSocketRef, JSON.parse(zoneSyncPayloadText));
  }, [routeSocketRef, zoneSyncPayloadText]);

  useEffect(() => {
    sendRouteChannelPayload(routeSocketRef, JSON.parse(surfaceSyncPayloadText));
  }, [routeSocketRef, surfaceSyncPayloadText]);

  useEffect(() => {
    if (!motionProfileTouchedRef.current) {
      motionProfileTouchedRef.current = true;
      return;
    }
    sendRouteChannelPayload(routeSocketRef, {
      type: "motion_profile",
      motion: {
        cruiseSpeedMps,
        payloadKg,
        batteryRange: batteryRangeMeters,
      },
    });
  }, [batteryRangeMeters, cruiseSpeedMps, payloadKg, routeSocketRef]);
};
