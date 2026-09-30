import { usePlannerMapExport } from "./usePlannerMapExport";
import { usePlannerRuntimeCommands } from "./usePlannerRuntimeCommands";

export const useDashboardPlannerRuntimeActions = ({
  batteryRangeMeters,
  mappingSurveyMode,
  optimizedRoute,
  payloadKg,
  plannerModel,
  points,
  routeSocketRef,
  setMapExportPromptOpen,
  setStatus,
  telemetry,
}) => {
  const runtimeCommands = usePlannerRuntimeCommands({
    batteryRangeMeters, mappingSurveyMode, optimizedRoute, payloadKg, plannerModel, points,
    routeSocketRef, setStatus, telemetry,
  });
  const mapExport = usePlannerMapExport({ setMapExportPromptOpen, setStatus, telemetry });
  return { ...runtimeCommands, ...mapExport };
};
