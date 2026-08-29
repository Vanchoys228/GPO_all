import { usePlannerGraphImport } from "./usePlannerGraphImport";
import { usePlannerRouteOptimization } from "./usePlannerRouteOptimization";
import { usePlannerRouteRebuild } from "./usePlannerRouteRebuild";
import { usePlannerRouteSender } from "./usePlannerRouteSender";

export const useDashboardPlannerRouteLifecycle = (input) => {
  const handleImportFile = usePlannerGraphImport(input);

  usePlannerRouteRebuild(input);

  const optimizeRoute = usePlannerRouteOptimization(input);
  const sendRoute = usePlannerRouteSender(input);

  return {
    handleImportFile,
    optimizeRoute,
    sendRoute,
  };
};
