import PlannerCanvas from "../../components/dashboard/PlannerCanvas";
import PlannerLeftSidebar from "../../components/dashboard/PlannerLeftSidebar";
import PlannerRightSidebar from "../../components/dashboard/PlannerRightSidebar";
import SidebarCollapseRail from "../../components/dashboard/SidebarCollapseRail";

export default function DashboardPlannerWorkspace({
  canvasProps,
  leftSidebarProps,
  plannerUiState,
  rightSidebarProps,
  setSidebarCollapsed,
}) {
  return (
    <div className="flex h-screen min-h-0 bg-stone-100 text-stone-900">
      {plannerUiState.leftCollapsed ? (
        <SidebarCollapseRail side="left" onExpand={() => setSidebarCollapsed("leftCollapsed", false)} />
      ) : (
        <PlannerLeftSidebar {...leftSidebarProps} onCollapse={() => setSidebarCollapsed("leftCollapsed", true)} />
      )}

      <PlannerCanvas {...canvasProps} />

      {plannerUiState.rightCollapsed ? (
        <SidebarCollapseRail side="right" onExpand={() => setSidebarCollapsed("rightCollapsed", false)} />
      ) : (
        <PlannerRightSidebar {...rightSidebarProps} onCollapse={() => setSidebarCollapsed("rightCollapsed", true)} />
      )}
    </div>
  );
}
