import { useCallback, useState } from "react";
import { loadPlannerUiState, savePlannerUiState } from "../../../lib/plannerUiState";

export const usePlannerSidebarState = () => {
  const [plannerUiState, setPlannerUiState] = useState(loadPlannerUiState);
  const setSidebarCollapsed = useCallback((key, collapsed) => {
    setPlannerUiState((previous) => {
      const next = { ...previous, [key]: collapsed };
      savePlannerUiState(next);
      return next;
    });
  }, []);
  return { plannerUiState, setSidebarCollapsed };
};
