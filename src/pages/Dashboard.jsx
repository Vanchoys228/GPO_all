import { useRef, useState } from "react";
import {
  ALGORITHM_OPTIONS,
  getAlgorithmFields,
  getDefaultAlgorithmParams,
} from "../lib/routeAlgorithms";
import { useMemo } from "react";
import {
  DEFAULT_SURFACE_ZONES,
} from "../lib/zonePlanner";
import {
  buildPlannerModel,
  INITIAL_ZONE,
} from "../lib/plannerModel";
import { analyzeRouteInfluence } from "../lib/energyModel";
import PlannerCanvas from "../components/dashboard/PlannerCanvas";
import PlannerLeftSidebar from "../components/dashboard/PlannerLeftSidebar";
import PlannerRightSidebar from "../components/dashboard/PlannerRightSidebar";
import SidebarCollapseRail from "../components/dashboard/SidebarCollapseRail";
import { loadPlannerUiState, savePlannerUiState } from "../lib/plannerUiState";
import { useRouteSocket } from "../features/planner/hooks/useRouteSocket";
import { useSolverHealth } from "../features/planner/hooks/useSolverHealth";
import { useTelemetrySocket } from "../features/planner/hooks/useTelemetrySocket";
import { usePlannerBridgeSync } from "../features/planner/hooks/usePlannerBridgeSync";
import { useRouteTiming } from "../features/planner/hooks/useRouteTiming";
import { usePlannerPointEditor } from "../features/planner/hooks/usePlannerPointEditor";
import { usePlannerLimitZoneEditor } from "../features/planner/hooks/usePlannerLimitZoneEditor";
import { usePlannerSurfaceZoneEditor } from "../features/planner/hooks/usePlannerSurfaceZoneEditor";
import { usePlannerRouteOptimization } from "../features/planner/hooks/usePlannerRouteOptimization";
import { usePlannerRouteSender } from "../features/planner/hooks/usePlannerRouteSender";
import { usePlannerRouteRebuild } from "../features/planner/hooks/usePlannerRouteRebuild";
import { usePlannerRuntimeCommands } from "../features/planner/hooks/usePlannerRuntimeCommands";
import { usePlannerGraphImport } from "../features/planner/hooks/usePlannerGraphImport";
import { usePlannerMapExport } from "../features/planner/hooks/usePlannerMapExport";
import { usePlannerEnergySettings } from "../features/planner/hooks/usePlannerEnergySettings";
import {
  DEFAULT_SURFACE_PROFILE_KEY,
  createInitialSurfaceZones,
} from "../features/planner/model/surfaceZones";
import {
  createEmptyRouteEnergyStats,
} from "../features/planner/model/routeEnergy";
import { MAPPING_SURVEY_MODES } from "../features/planner/model/runtimeCommands";
import { buildPlannerSyncPayloads } from "../features/planner/model/plannerSync";

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
  const [plannerUiState, setPlannerUiState] = useState(loadPlannerUiState);
  const [limitZones, setLimitZones] = useState([INITIAL_ZONE]);
  const [activeLimitZoneId, setActiveLimitZoneId] = useState(INITIAL_ZONE.id);
  const [nextZoneNumber, setNextZoneNumber] = useState(2);
  const [mappingSurveyMode, setMappingSurveyMode] = useState(
    MAPPING_SURVEY_MODES[0].key
  );
  const [algorithmParams, setAlgorithmParams] = useState(() =>
    Object.fromEntries(
      ALGORITHM_OPTIONS.map((option) => [
        option.key,
        getDefaultAlgorithmParams(option.key),
      ])
    )
  );

  const plannerModel = buildPlannerModel({
    points,
    limitZones,
    optimizedRoute,
    activeLimitZoneId,
    surfaceZones,
  });
  const activeSurfaceZone =
    surfaceZones.find((zone) => zone.id === activeSurfaceZoneId) || surfaceZones[0] || null;
  const algorithmFields = getAlgorithmFields(algorithmKey);
  const selectedAlgorithmParams =
    algorithmParams[algorithmKey] || getDefaultAlgorithmParams(algorithmKey);
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

  const handleImportFile = usePlannerGraphImport({
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
    setNextSurfaceZoneNumber,
    setNextZoneNumber,
    setOptimizedRoute,
    setPoints,
    setRouteEnergyStats,
    setRouteSeed,
    setRouteTaskKey,
    setStatus,
    setSurfaceZones,
  });

  usePlannerRouteRebuild({
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
    setEnergyWarning,
    setOptimizedRoute,
    setRouteEnergyStats,
    setStatus,
    startRouteTiming,
    surfaceSyncPayloadText,
    surfaceZones: plannerModel.surfaceZones,
    zoneSyncPayloadText,
  });

  const clearRouteState = ({ dropSolvedRoute = true } = {}) => {
    setExpandedPoint(null);
    setHoveredPointIndex(null);
    if (dropSolvedRoute) {
      setRouteSeed([]);
      setOptimizedRoute([]);
      setEnergyWarning("");
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      resetRouteTiming();
    }
  };

  const {
    clearZone,
    createZone,
    removeZone,
    resetZones,
    selectZone,
    toggleZoneClosed,
  } = usePlannerLimitZoneEditor({
    activeLimitZoneId,
    clearRouteState,
    limitZones,
    nextZoneNumber,
    points,
    setActiveLimitZoneId,
    setActivePointKind,
    setLimitZones,
    setNextZoneNumber,
    setPoints,
    setStatus,
    zoneEntries: plannerModel.zoneEntries,
  });

  const { clearAllSurfaceZones, clearSurfaceZone, createSurfaceZone, removeSurfaceZone, selectSurfaceZone, toggleSurfaceZoneClosed, updateActiveSurfaceProfile } =
    usePlannerSurfaceZoneEditor({
      activeSurfaceProfileKey,
      activeSurfaceZoneId,
      clearRouteState,
      nextSurfaceZoneNumber,
      setActivePointKind,
      setActiveSurfaceProfileKey,
      setActiveSurfaceZoneId,
      setNextSurfaceZoneNumber,
      setStatus,
      setSurfaceZones,
      surfaceZones,
    });

  const updateAlgorithmParam = (field, rawValue) => {
    const parsed = field.integer ? parseInt(rawValue, 10) : parseFloat(rawValue);
    if (!Number.isFinite(parsed)) return;

    setAlgorithmParams((prev) => ({
      ...prev,
      [algorithmKey]: {
        ...getDefaultAlgorithmParams(algorithmKey),
        ...prev[algorithmKey],
        [field.key]: parsed,
      },
    }));
    clearRouteState();
  };

  const {
    addPointFromCanvas,
    clearPoints,
    deletePoint,
    finishDragging,
    handleCanvasMouseDown,
    handleCanvasMouseMove,
    updatePointTask,
  } = usePlannerPointEditor({
    activeLimitZoneId,
    activePointKind,
    activeSurfaceProfileKey,
    activeSurfaceZone,
    activeZone: plannerModel.activeZone,
    activeZoneName: plannerModel.activeZoneName,
    canvasRef,
    clearRouteState,
    points,
    resetZones,
    setPoints,
    setStatus,
    setSurfaceZones,
  });

  const handleRouteTaskChange = (nextTaskKey) => {
    setRouteTaskKey(nextTaskKey);
    clearRouteState();
    setStatus("");
    setEnergyWarning("");
  };

  const handleAlgorithmChange = (nextAlgorithmKey) => {
    setAlgorithmKey(nextAlgorithmKey);
    clearRouteState();
    setStatus("");
    setEnergyWarning("");
  };

  const optimizeRoute = usePlannerRouteOptimization({
    algorithmKey,
    batteryRangeMeters,
    energyOptions,
    isOptimizing,
    plannerModel,
    routeTaskKey,
    selectedAlgorithmParams,
    setEnergyWarning,
    setIsOptimizing,
    setOptimizedRoute,
    setRouteEnergyStats,
    setRouteSeed,
    setStatus,
    telemetry,
  });

  const sendRoute = usePlannerRouteSender({
    algorithmKey,
    batteryRangeMeters,
    cruiseSpeedMps,
    energyOptions,
    optimizedRoute,
    payloadKg,
    plannerModel,
    routeSeed,
    routeSocketRef: routeWsRef,
    routeTaskKey,
    selectedAlgorithmParams,
    setEnergyWarning,
    setRouteEnergyStats,
    setStatus,
    startRouteTiming,
  });

  const { addRandomObstacle, startMappingSurvey } = usePlannerRuntimeCommands({
    batteryRangeMeters,
    mappingSurveyMode,
    optimizedRoute,
    payloadKg,
    plannerModel,
    points,
    routeSocketRef: routeWsRef,
    setStatus,
    telemetry,
  });

  const { exportMapImage, requestMapExport } = usePlannerMapExport({
    setMapExportPromptOpen,
    setStatus,
    telemetry,
  });

  const setSidebarCollapsed = (key, collapsed) => {
    setPlannerUiState((prev) => {
      const next = { ...prev, [key]: collapsed };
      savePlannerUiState(next);
      return next;
    });
  };

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
