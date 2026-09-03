import DashboardPlannerWorkspace from "./DashboardPlannerWorkspace";
import { useDashboardPlannerActions } from "./hooks/useDashboardPlannerActions";
import { useDashboardPlannerDerivedState } from "./hooks/useDashboardPlannerDerivedState";
import { useDashboardPlannerRuntime } from "./hooks/useDashboardPlannerRuntime";
import { useDashboardPlannerState } from "./hooks/useDashboardPlannerState";
import {
  createCanvasProps,
  createLeftSidebarProps,
  createRightSidebarProps,
} from "./model/dashboardPlannerWorkspaceProps";

export default function DashboardPlannerController() {
  const state = useDashboardPlannerState();
  const runtime = useDashboardPlannerRuntime();
  const derived = useDashboardPlannerDerivedState(state, runtime);
  const actions = useDashboardPlannerActions(state, runtime, derived);
  const workspaceContext = { state, runtime, derived, actions };

  return (
    <DashboardPlannerWorkspace
      plannerUiState={state.sidebar.plannerUiState}
      setSidebarCollapsed={state.sidebar.setSidebarCollapsed}
      canvasProps={createCanvasProps(workspaceContext)}
      leftSidebarProps={createLeftSidebarProps(workspaceContext)}
      rightSidebarProps={createRightSidebarProps(workspaceContext)}
    />
  );
}
