import { useMissionTracking } from "./useMissionTracking";
import { useRouteSocket } from "./useRouteSocket";
import { useRouteTiming } from "./useRouteTiming";
import { useSolverHealth } from "./useSolverHealth";
import { useTelemetrySocket } from "./useTelemetrySocket";

export const useDashboardPlannerRuntime = () => {
  const { connected: telemetryWsUp, telemetry } = useTelemetrySocket();
  const { connected: routeWsUp, socketRef: routeWsRef } = useRouteSocket();
  const solverApiUp = useSolverHealth();
  const routeTiming = useRouteTiming(telemetry.navigation);

  const mission = useMissionTracking({onStarted:routeTiming.start});
  return {
    mission,
    telemetryWsUp,
    telemetry,
    routeWsUp,
    routeWsRef,
    solverApiUp,
    routeTiming,
  };
};
