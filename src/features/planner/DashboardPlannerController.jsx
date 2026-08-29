import { useRef, useState } from "react";
import {
  DEFAULT_SURFACE_ZONES,
} from "../../lib/zonePlanner";
import { useMemo } from "react";
import {
  buildPlannerModel,
  INITIAL_ZONE,
} from "../../lib/plannerModel";
import { analyzeRouteInfluence } from "../../lib/energyModel";
import PlannerCanvas from "../../components/dashboard/PlannerCanvas";
import PlannerLeftSidebar from "../../components/dashboard/PlannerLeftSidebar";
import PlannerRightSidebar from "../../components/dashboard/PlannerRightSidebar";
import SidebarCollapseRail from "../../components/dashboard/SidebarCollapseRail";
import { useRouteSocket } from "./hooks/useRouteSocket";
import { useSolverHealth } from "./hooks/useSolverHealth";
import { useTelemetrySocket } from "./hooks/useTelemetrySocket";
import { usePlannerBridgeSync } from "./hooks/usePlannerBridgeSync";
import { useRouteTiming } from "./hooks/useRouteTiming";
import { usePlannerEnergySettings } from "./hooks/usePlannerEnergySettings";
import { usePlannerRouteSelection } from "./hooks/usePlannerRouteSelection";
import { usePlannerSidebarState } from "./hooks/usePlannerSidebarState";
import { useDashboardPlannerRuntimeActions } from "./hooks/useDashboardPlannerRuntimeActions";
import { useDashboardPlannerEditors } from "./hooks/useDashboardPlannerEditors";
import { useDashboardPlannerRouteLifecycle } from "./hooks/useDashboardPlannerRouteLifecycle";
import {
  DEFAULT_SURFACE_PROFILE_KEY,
  createInitialSurfaceZones,
} from "./model/surfaceZones";
import {
  createEmptyRouteEnergyStats,
} from "./model/routeEnergy";
import { MAPPING_SURVEY_MODES } from "./model/runtimeCommands";
import { buildPlannerSyncPayloads } from "./model/plannerSync";
import { createDashboardAlgorithmParams } from "./model/dashboardAlgorithmParams";
import { createDashboardPlannerViewModel } from "./model/dashboardPlannerViewModel";

export default function Dashboard() {
  const canvasRef = useRef(null);
  const { connected: telemetryWsUp, telemetry } = useTelemetrySocket();
  const { connected: routeWsUp, socketRef: routeWsRef } = useRouteSocket();
  const solverApiUp = useSolverHealth();
  const {
    avoidanceTimeSec: routeAvoidanceTimeSec,
    display: routeTimingDisplay,
    offRouteActive: routeOffRouteActive,
    reset: resetRouteTiming,
    start: startRouteTiming,
  } = useRouteTiming(telemetry.navigation);

  const [points, setPoints] = useState([]);
  const [routeSeed, setRouteSeed] = useState([]);
  const [optimizedRoute, setOptimizedRoute] = useState([]);
  const [status, setStatus] = useState("");
  const [energyWarning, setEnergyWarning] = useState("");
  const [expandedPoint, setExpandedPoint] = useState(null);
  const [hoveredPointIndex, setHoveredPointIndex] = useState(null);
  const [isOptimizing, setIsOptimizing] = useState(false);
  const [routeTaskKey, setRouteTaskKey] = useState("tsp");
  const [algorithmKey, setAlgorithmKey] = useState("ga_tabu");
  const [activePointKind, setActivePointKind] = useState("visit");
  const [surfaceZones, setSurfaceZones] = useState(createInitialSurfaceZones);
  const [activeSurfaceZoneId, setActiveSurfaceZoneId] = useState(
    () => `surface-zone-${DEFAULT_SURFACE_ZONES.length + 1}`
  );
  const [activeSurfaceProfileKey, setActiveSurfaceProfileKey] = useState(
    DEFAULT_SURFACE_PROFILE_KEY
  );
  const [nextSurfaceZoneNumber, setNextSurfaceZoneNumber] = useState(
    () => DEFAULT_SURFACE_ZONES.length + 2
  );
  const [mapExportPromptOpen, setMapExportPromptOpen] = useState(false);
  const [routeEnergyStats, setRouteEnergyStats] = useState(createEmptyRouteEnergyStats);
  const {
    batteryRangeInput,
    batteryRangeMeters,
    cruiseSpeedInput,
    cruiseSpeedMps,
    handleBatteryRangeBlur,
    handleBatteryRangeChange,
    handleCruiseSpeedBlur,
    handleCruiseSpeedChange,
    handlePayloadBlur,
    handlePayloadChange,
    payloadInput,
    payloadKg,
  } = usePlannerEnergySettings({
    setEnergyWarning,
    setExpandedPoint,
    setHoveredPointIndex,
    setRouteEnergyStats,
  });
  const { plannerUiState, setSidebarCollapsed } = usePlannerSidebarState();
  const [limitZones, setLimitZones] = useState([INITIAL_ZONE]);
  const [activeLimitZoneId, setActiveLimitZoneId] = useState(INITIAL_ZONE.id);
  const [nextZoneNumber, setNextZoneNumber] = useState(2);
  const [mappingSurveyMode, setMappingSurveyMode] = useState(
    MAPPING_SURVEY_MODES[0].key
  );
  const [algorithmParams, setAlgorithmParams] = useState(createDashboardAlgorithmParams);

  const plannerModel = buildPlannerModel({
    points,
    limitZones,
    optimizedRoute,
    activeLimitZoneId,
    surfaceZones,
  });
  const { activeSurfaceZone, algorithmFields, selectedAlgorithmParams } =
    createDashboardPlannerViewModel({
      algorithmKey,
      algorithmParams,
      activeSurfaceZoneId,
      surfaceZones,
    });
  const {
    chargePointsRoutingText,
    previewPolygonRoutingText,
    surfaceSyncPayloadText,
    zoneSyncPayloadText,
  } = buildPlannerSyncPayloads(plannerModel);
  usePlannerBridgeSync({
    batteryRangeMeters,
    cruiseSpeedMps,
    payloadKg,
    routeSocketRef: routeWsRef,
    surfaceSyncPayloadText,
    zoneSyncPayloadText,
  });
  const energyOptions = useMemo(
    () => ({
      speedMps: cruiseSpeedMps,
      payloadKg,
    }),
    [cruiseSpeedMps, payloadKg]
  );
  const telemetryForSidebar = useMemo(
    () => ({
      ...telemetry,
      navigation: {
        ...telemetry.navigation,
        avoidanceTimeSec: routeAvoidanceTimeSec,
        offRouteActive: routeOffRouteActive,
      },
    }),
    [routeAvoidanceTimeSec, routeOffRouteActive, telemetry]
  );
  const routeInfluenceRows = useMemo(
    () =>
      analyzeRouteInfluence(optimizedRoute, {
        surfaceZones: plannerModel.surfaceZones,
        speedMps: cruiseSpeedMps,
        payloadKg,
        stationStopCount: routeEnergyStats.stationStopCount,
        plannedTimeSec: routeEnergyStats.estimatedTimeSec,
        actualTimeSec: routeTimingDisplay.actualTimeSec,
        avoidanceTimeSec: routeAvoidanceTimeSec,
      }),
    [
      cruiseSpeedMps,
      optimizedRoute,
      payloadKg,
      plannerModel.surfaceZones,
      routeAvoidanceTimeSec,
      routeEnergyStats.estimatedTimeSec,
      routeEnergyStats.stationStopCount,
      routeTimingDisplay.actualTimeSec,
    ]
  );

  const { handleImportFile, optimizeRoute, sendRoute } = useDashboardPlannerRouteLifecycle({
    resetRouteTiming,
    setActiveLimitZoneId,
    setActivePointKind,
    setActiveSurfaceProfileKey,
    setActiveSurfaceZoneId,
    setAlgorithmKey,
    setEnergyWarning,
    setExpandedPoint,
    setHoveredPointIndex,
    setLimitZones,
    setIsOptimizing,
    setNextSurfaceZoneNumber,
    setNextZoneNumber,
    setOptimizedRoute,
    setPoints,
    setRouteEnergyStats,
    setRouteSeed,
    setRouteTaskKey,
    setStatus,
    setSurfaceZones,
    algorithmKey,
    batteryRangeMeters,
    chargePointsRoutingText,
    cruiseSpeedMps,
    energyOptions,
    payloadKg,
    previewPolygonRoutingText,
    routeSeed,
    routeSocketRef: routeWsRef,
    routeTaskKey,
    selectedAlgorithmParams,
    startRouteTiming,
    surfaceSyncPayloadText,
    surfaceZones: plannerModel.surfaceZones,
    telemetry,
    zoneSyncPayloadText,
  });

  const { clearRouteState, handleAlgorithmChange, handleRouteTaskChange, updateAlgorithmParam } =
    usePlannerRouteSelection({
      algorithmKey, resetRouteTiming, setAlgorithmKey, setAlgorithmParams, setEnergyWarning,
      setExpandedPoint, setHoveredPointIndex, setOptimizedRoute, setRouteEnergyStats,
      setRouteSeed, setRouteTaskKey, setStatus,
    });

  const {
    addPointFromCanvas, clearAllSurfaceZones, clearPoints, clearSurfaceZone, clearZone,
    createSurfaceZone, createZone, deletePoint, finishDragging, handleCanvasMouseDown,
    handleCanvasMouseMove, removeSurfaceZone, removeZone, selectSurfaceZone, selectZone,
    toggleSurfaceZoneClosed, toggleZoneClosed, updateActiveSurfaceProfile, updatePointTask,
  } = useDashboardPlannerEditors({
    activeLimitZoneId, activePointKind, activeSurfaceProfileKey, activeSurfaceZone,
    activeSurfaceZoneId, canvasRef, clearRouteState, limitZones, nextSurfaceZoneNumber,
    nextZoneNumber, plannerModel, points, setActiveLimitZoneId, setActivePointKind,
    setActiveSurfaceProfileKey, setActiveSurfaceZoneId, setLimitZones, setNextSurfaceZoneNumber,
    setNextZoneNumber, setPoints, setStatus, setSurfaceZones, surfaceZones,
  });

  const { addRandomObstacle, exportMapImage, requestMapExport, startMappingSurvey } =
    useDashboardPlannerRuntimeActions({
      batteryRangeMeters, mappingSurveyMode, optimizedRoute, payloadKg, plannerModel, points,
      routeSocketRef: routeWsRef, setMapExportPromptOpen, setStatus, telemetry,
    });

  return (
    <div className="flex h-screen min-h-0 bg-stone-100 text-stone-900">
      {plannerUiState.leftCollapsed ? (
        <SidebarCollapseRail
          side="left"
          onExpand={() => setSidebarCollapsed("leftCollapsed", false)}
        />
      ) : (
        <PlannerLeftSidebar
          onCollapse={() => setSidebarCollapsed("leftCollapsed", true)}
          onImportFile={handleImportFile}
          activePointKind={activePointKind}
          onActivePointKindChange={setActivePointKind}
          onClearVisitPoints={() => clearPoints("visit")}
          onClearChargePoints={() => clearPoints("charge")}
          onClearLimitPoints={() => clearPoints("limit")}
          routeTaskKey={routeTaskKey}
          onRouteTaskChange={handleRouteTaskChange}
          algorithmKey={algorithmKey}
          onAlgorithmChange={handleAlgorithmChange}
          status={status}
          energyWarning={energyWarning}
          routeBlocked={plannerModel.routeBlocked}
          algorithmFields={algorithmFields}
          selectedAlgorithmParams={selectedAlgorithmParams}
          onAlgorithmParamChange={updateAlgorithmParam}
          isOptimizing={isOptimizing}
          onOptimizeRoute={optimizeRoute}
          onSendRoute={sendRoute}
          onAddRandomObstacle={addRandomObstacle}
          onClearAll={() => clearPoints()}
          hasRoute={optimizedRoute.length > 0}
          routeLength={plannerModel.routeLength}
          visitCount={plannerModel.visitEntries.length}
          chargeCount={plannerModel.chargeEntries.length}
          zoneCount={plannerModel.zoneEntries.length}
          polygonCount={plannerModel.polygons.length}
          adjustedVisitCount={plannerModel.adjustedVisits.length}
          activeZoneName={plannerModel.activeZoneName}
          batteryRangeInput={batteryRangeInput}
          onBatteryRangeChange={handleBatteryRangeChange}
          onBatteryRangeBlur={handleBatteryRangeBlur}
          cruiseSpeedMps={cruiseSpeedMps}
          cruiseSpeedInput={cruiseSpeedInput}
          onCruiseSpeedChange={handleCruiseSpeedChange}
          onCruiseSpeedBlur={handleCruiseSpeedBlur}
          payloadKg={payloadKg}
          payloadInput={payloadInput}
          onPayloadChange={handlePayloadChange}
          onPayloadBlur={handlePayloadBlur}
          routeEnergyStats={routeEnergyStats}
          routeInfluenceRows={routeInfluenceRows}
          routeTiming={routeTimingDisplay}
          surfaceZones={surfaceZones}
          activeSurfaceZoneId={activeSurfaceZoneId}
          activeSurfaceZone={activeSurfaceZone}
          activeSurfaceProfileKey={activeSurfaceProfileKey}
          onActiveSurfaceProfileChange={updateActiveSurfaceProfile}
          onCreateSurfaceZone={createSurfaceZone}
          onSelectSurfaceZone={selectSurfaceZone}
          onToggleSurfaceZoneClosed={toggleSurfaceZoneClosed}
          onClearSurfaceZone={clearSurfaceZone}
          onRemoveSurfaceZone={removeSurfaceZone}
          onClearAllSurfaceZones={clearAllSurfaceZones}
        />
      )}

      <PlannerCanvas
        canvasRef={canvasRef}
        plannerModel={plannerModel}
        optimizedRoute={optimizedRoute}
        hoveredPointIndex={hoveredPointIndex}
        telemetry={telemetry}
        onCanvasClick={addPointFromCanvas}
        onCanvasMouseDown={handleCanvasMouseDown}
        onCanvasMouseMove={handleCanvasMouseMove}
        onCanvasMouseUp={finishDragging}
        onCanvasMouseLeave={finishDragging}
      />

      {plannerUiState.rightCollapsed ? (
        <SidebarCollapseRail
          side="right"
          onExpand={() => setSidebarCollapsed("rightCollapsed", false)}
        />
      ) : (
        <PlannerRightSidebar
          onCollapse={() => setSidebarCollapsed("rightCollapsed", true)}
          activeZone={plannerModel.activeZone}
          activeZoneName={plannerModel.activeZoneName}
          activeLimitZoneId={activeLimitZoneId}
          zoneEntries={plannerModel.zoneEntries}
          visitEntries={plannerModel.visitEntries}
          chargeEntries={plannerModel.chargeEntries}
          plannedVisitEntries={plannerModel.plannedVisitEntries}
          expandedPoint={expandedPoint}
          hoveredPointIndex={hoveredPointIndex}
          visitsInsideLimitCount={plannerModel.visitsInsideLimit.length}
          polygonCount={plannerModel.polygons.length}
          adjustedVisitCount={plannerModel.adjustedVisits.length}
          routeBlocked={plannerModel.routeBlocked}
          telemetry={telemetryForSidebar}
          telemetryWsUp={telemetryWsUp}
          routeWsUp={routeWsUp}
          solverApiUp={solverApiUp}
          mappingSurveyMode={mappingSurveyMode}
          mappingSurveyModes={MAPPING_SURVEY_MODES}
          onMappingSurveyModeChange={setMappingSurveyMode}
          mapExportPromptOpen={mapExportPromptOpen}
          onStartMappingSurvey={startMappingSurvey}
          onRequestMapExport={requestMapExport}
          onExportMapVariant={exportMapImage}
          onCancelMapExport={() => setMapExportPromptOpen(false)}
          onCreateZone={createZone}
          onSelectZone={selectZone}
          onToggleZoneClosed={toggleZoneClosed}
          onClearZone={clearZone}
          onRemoveZone={removeZone}
          onToggleExpandedPoint={setExpandedPoint}
          onHoverPoint={setHoveredPointIndex}
          onDeletePoint={deletePoint}
          onUpdatePointTask={updatePointTask}
        />
      )}
    </div>
  );
}
