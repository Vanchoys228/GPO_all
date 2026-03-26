import { useEffect, useRef, useState } from "react";
import {
  ALGORITHM_OPTIONS,
  getAlgorithmFields,
  getAlgorithmLabel,
  getDefaultAlgorithmParams,
  getTaskLabel,
  probeNativeSolver,
  solveRouteWithNativeAlgorithm,
} from "../lib/routeAlgorithms";
import {
  DEFAULT_POINT_TASK,
  buildObstacleAwareRoute,
  canvasToWorld,
  isInsideMap,
  routeCrossesAnyLimitPolygon,
  sanitizeRouteForController,
  worldToCanvas,
} from "../lib/zonePlanner";
import {
  decodeWsData,
  INITIAL_TELEMETRY,
  normalizeTelemetry,
  ROUTE_WS_URL,
  TELEMETRY_WS_URL,
} from "../lib/dashboardTelemetry";
import {
  buildPlannerModel,
  DRAG_HIT_RADIUS,
  getRouteAnchor,
  INITIAL_ZONE,
  rotateClosedRouteToNearestPoint,
} from "../lib/plannerModel";
import PlannerCanvas from "../components/dashboard/PlannerCanvas";
import PlannerLeftSidebar from "../components/dashboard/PlannerLeftSidebar";
import PlannerRightSidebar from "../components/dashboard/PlannerRightSidebar";

const getCanvasEventPosition = (canvas, event) => {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;

  return {
    x: (event.clientX - rect.left) * scaleX,
    y: (event.clientY - rect.top) * scaleY,
  };
};

const buildLimitZonePayload = (polygons) => ({
  type: "limit_zones",
  zones: polygons.map((zone) => ({
    id: zone.id,
    name: zone.name,
    points: zone.points.map((point) => ({
      x: point.x,
      y: point.y,
    })),
  })),
});

const sendRouteChannelPayload = (routeWsRef, payload, { onSent, onError } = {}) => {
  const text = JSON.stringify(payload);
  const ws = routeWsRef.current;
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(text);
    if (onSent) onSent();
    return;
  }

  const tempSocket = new WebSocket(ROUTE_WS_URL);
  let settled = false;

  const closeTempSocket = () => {
    try {
      tempSocket.close();
    } catch {
      // Ignore close failures.
    }
  };

  tempSocket.onopen = () => {
    if (settled) return;
    settled = true;
    tempSocket.send(text);
    if (onSent) onSent();
    window.setTimeout(closeTempSocket, 80);
  };

  tempSocket.onerror = () => {
    if (settled) return;
    settled = true;
    if (onError) onError();
    closeTempSocket();
  };

  tempSocket.onclose = () => {
    if (settled) return;
    settled = true;
    if (onError) onError();
  };
};

export default function Dashboard() {
  const canvasRef = useRef(null);
  const routeWsRef = useRef(null);
  const lastAutoRouteZoneSyncRef = useRef(null);
  const dragStateRef = useRef({
    pointIndex: null,
    moved: false,
    preventClick: false,
  });

  const [points, setPoints] = useState([]);
  const [routeSeed, setRouteSeed] = useState([]);
  const [optimizedRoute, setOptimizedRoute] = useState([]);
  const [status, setStatus] = useState("");
  const [expandedPoint, setExpandedPoint] = useState(null);
  const [hoveredPointIndex, setHoveredPointIndex] = useState(null);
  const [telemetryWsUp, setTelemetryWsUp] = useState(false);
  const [routeWsUp, setRouteWsUp] = useState(false);
  const [solverApiUp, setSolverApiUp] = useState(false);
  const [isOptimizing, setIsOptimizing] = useState(false);
  const [telemetry, setTelemetry] = useState(INITIAL_TELEMETRY);
  const [routeTaskKey, setRouteTaskKey] = useState("tsp");
  const [algorithmKey, setAlgorithmKey] = useState("ga_tabu");
  const [activePointKind, setActivePointKind] = useState("visit");
  const [limitZones, setLimitZones] = useState([INITIAL_ZONE]);
  const [activeLimitZoneId, setActiveLimitZoneId] = useState(INITIAL_ZONE.id);
  const [nextZoneNumber, setNextZoneNumber] = useState(2);
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
  });
  const algorithmFields = getAlgorithmFields(algorithmKey);
  const selectedAlgorithmParams =
    algorithmParams[algorithmKey] || getDefaultAlgorithmParams(algorithmKey);
  const zoneSyncPayloadText = JSON.stringify(
    buildLimitZonePayload(
      plannerModel.polygons.map((zone) => ({
        ...zone,
        points: zone.points.map((point) => ({
          x: Number(point.x.toFixed(4)),
          y: Number(point.y.toFixed(4)),
        })),
      }))
    )
  );
  const previewPolygonRoutingText = JSON.stringify(
    plannerModel.previewPolygons.map((zone) => ({
      id: zone.id,
      name: zone.name,
      points: zone.points.map((point) => ({
        x: Number(point.x.toFixed(4)),
        y: Number(point.y.toFixed(4)),
      })),
    }))
  );

  useEffect(() => {
    let closed = false;
    let ws = null;

    const connect = () => {
      if (closed) return;
      ws = new WebSocket(TELEMETRY_WS_URL);
      ws.onopen = () => setTelemetryWsUp(true);
      ws.onmessage = async (message) => {
        try {
          const payload = JSON.parse(await decodeWsData(message.data));
          setTelemetry((prev) => normalizeTelemetry(payload, prev) || prev);
        } catch {
          // Ignore malformed payloads.
        }
      };
      ws.onclose = () => {
        setTelemetryWsUp(false);
        if (!closed) setTimeout(connect, 1000);
      };
      ws.onerror = () => setTelemetryWsUp(false);
    };

    connect();
    return () => {
      closed = true;
      if (ws) ws.close();
    };
  }, []);

  useEffect(() => {
    let cancelled = false;
    let timer = 0;

    const checkSolver = async () => {
      try {
        const payload = await probeNativeSolver();
        if (!cancelled) setSolverApiUp(Boolean(payload?.solverAvailable));
      } catch {
        if (!cancelled) setSolverApiUp(false);
      } finally {
        if (!cancelled) timer = window.setTimeout(checkSolver, 2500);
      }
    };

    checkSolver();
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
    };
  }, []);

  useEffect(() => {
    let closed = false;

    const connect = () => {
      if (closed) return;
      const ws = new WebSocket(ROUTE_WS_URL);
      routeWsRef.current = ws;
      ws.onopen = () => setRouteWsUp(true);
      ws.onclose = () => {
        setRouteWsUp(false);
        routeWsRef.current = null;
        if (!closed) setTimeout(connect, 1000);
      };
      ws.onerror = () => setRouteWsUp(false);
    };

    connect();
    return () => {
      closed = true;
      setRouteWsUp(false);
      if (routeWsRef.current) routeWsRef.current.close();
    };
  }, []);

  useEffect(() => {
    sendRouteChannelPayload(routeWsRef, JSON.parse(zoneSyncPayloadText));
  }, [zoneSyncPayloadText]);

  useEffect(() => {
    if (!routeSeed.length) {
      setOptimizedRoute([]);
      return;
    }

    const previewPolygons = JSON.parse(previewPolygonRoutingText);
    const nextRoute =
      buildObstacleAwareRoute(routeSeed, previewPolygons) ||
      routeSeed.map((point) => ({ ...point }));
    setOptimizedRoute(nextRoute);
  }, [routeSeed, previewPolygonRoutingText]);

  useEffect(() => {
    if (lastAutoRouteZoneSyncRef.current === zoneSyncPayloadText) return;
    lastAutoRouteZoneSyncRef.current = zoneSyncPayloadText;
    if (routeSeed.length < 2) return;

    const controllerRoute = buildObstacleAwareRoute(routeSeed, plannerModel.polygons) || routeSeed;
    const routeForController = sanitizeRouteForController(controllerRoute);
    if (routeForController.length < 2) return;

    const payload = {
      type: "route",
      algorithm: {
        key: algorithmKey,
        task: routeTaskKey,
        params: selectedAlgorithmParams,
      },
      route: routeForController.map((point) => ({ x: point.x, y: point.y })),
    };

    sendRouteChannelPayload(routeWsRef, payload, {
      onSent: () => {
        setStatus(`Маршрут обновлён после изменения ограничивающих зон (${routeForController.length} точек).`);
      },
    });
  }, [
    algorithmKey,
    plannerModel.polygons,
    routeSeed,
    routeTaskKey,
    selectedAlgorithmParams,
    zoneSyncPayloadText,
  ]);

  const resetZones = () => {
    setLimitZones([INITIAL_ZONE]);
    setActiveLimitZoneId(INITIAL_ZONE.id);
    setNextZoneNumber(2);
  };

  const clearRouteState = ({ dropSolvedRoute = true } = {}) => {
    setExpandedPoint(null);
    setHoveredPointIndex(null);
    if (dropSolvedRoute) {
      setRouteSeed([]);
      setOptimizedRoute([]);
    }
  };

  const createZone = () => {
    const zone = {
      id: `zone-${nextZoneNumber}`,
      name: `Р—РѕРЅР° ${nextZoneNumber}`,
      closed: false,
    };
    setLimitZones((prev) => [...prev, zone]);
    setActiveLimitZoneId(zone.id);
    setNextZoneNumber((prev) => prev + 1);
    setActivePointKind("limit");
    setStatus(`РЎРѕР·РґР°РЅР° ${zone.name}.`);
  };

  const selectZone = (zoneId) => {
    setActiveLimitZoneId(zoneId);
    setActivePointKind("limit");
  };

  const toggleZoneClosed = (zoneId) => {
    const target = plannerModel.zoneEntries.find((zone) => zone.id === zoneId);
    if (!target) return;

    if (!target.closed && target.points.length < 3) {
      setStatus("Р§С‚РѕР±С‹ Р·Р°РјРєРЅСѓС‚СЊ Р·РѕРЅСѓ, РЅСѓР¶РЅРѕ РјРёРЅРёРјСѓРј С‚СЂРё С‚РѕС‡РєРё.");
      return;
    }

    setLimitZones((prev) =>
      prev.map((zone) =>
        zone.id === zoneId ? { ...zone, closed: !zone.closed } : zone
      )
    );
    clearRouteState({ dropSolvedRoute: false });
    setStatus(
      target.closed
        ? `${target.name} РѕС‚РєСЂС‹С‚Р° РґР»СЏ СЂРµРґР°РєС‚РёСЂРѕРІР°РЅРёСЏ.`
        : `${target.name} Р·Р°РјРєРЅСѓС‚Р°.`
    );
  };

  const clearZone = (zoneId) => {
    setPoints((prev) =>
      prev.filter((point) => point.kind !== "limit" || point.zoneId !== zoneId)
    );
    setLimitZones((prev) =>
      prev.map((zone) => (zone.id === zoneId ? { ...zone, closed: false } : zone))
    );
    clearRouteState({ dropSolvedRoute: false });
    setStatus("РўРѕС‡РєРё РІС‹Р±СЂР°РЅРЅРѕР№ Р·РѕРЅС‹ РѕС‡РёС‰РµРЅС‹.");
  };

  const removeZone = (zoneId) => {
    if (limitZones.length === 1) {
      clearZone(zoneId);
      return;
    }

    const nextZones = limitZones.filter((zone) => zone.id !== zoneId);
    setLimitZones(nextZones);
    setPoints((prev) =>
      prev.filter((point) => point.kind !== "limit" || point.zoneId !== zoneId)
    );
    if (activeLimitZoneId === zoneId) setActiveLimitZoneId(nextZones[0].id);
    clearRouteState({ dropSolvedRoute: false });
    setStatus("РћРіСЂР°РЅРёС‡РёРІР°СЋС‰Р°СЏ Р·РѕРЅР° СѓРґР°Р»РµРЅР°.");
  };

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

  const getPointIndexAtCanvasPosition = (canvasX, canvasY) => {
    for (let index = points.length - 1; index >= 0; index -= 1) {
      const point = points[index];
      const rendered = worldToCanvas(point.x, point.y);
      if (Math.hypot(rendered.x - canvasX, rendered.y - canvasY) <= DRAG_HIT_RADIUS) {
        return index;
      }
    }

    return -1;
  };

  const movePoint = (pointIndex, nextPoint) => {
    if (!isInsideMap(nextPoint)) return false;

    const currentPoint = points[pointIndex];
    if (!currentPoint) return false;

    setPoints((prev) =>
      prev.map((point, index) =>
        index === pointIndex ? { ...point, x: nextPoint.x, y: nextPoint.y } : point
      )
    );
    if (currentPoint.kind === "visit") clearRouteState();
    else clearRouteState({ dropSolvedRoute: false });
    return true;
  };

  const handleCanvasMouseDown = (event) => {
    if (!canvasRef.current) return;

    const canvasPoint = getCanvasEventPosition(canvasRef.current, event);
    const pointIndex = getPointIndexAtCanvasPosition(canvasPoint.x, canvasPoint.y);

    if (pointIndex < 0) return;

    dragStateRef.current = {
      pointIndex,
      moved: false,
      preventClick: false,
    };
  };

  const handleCanvasMouseMove = (event) => {
    if (!canvasRef.current) return;
    const { pointIndex } = dragStateRef.current;
    if (pointIndex === null) return;

    const canvasPoint = getCanvasEventPosition(canvasRef.current, event);
    const nextPoint = canvasToWorld(canvasPoint.x, canvasPoint.y);
    const moved = movePoint(pointIndex, nextPoint);
    if (moved) dragStateRef.current.moved = true;
  };

  const finishDragging = () => {
    const { pointIndex, moved } = dragStateRef.current;
    if (pointIndex === null) return;

    dragStateRef.current = {
      pointIndex: null,
      moved: false,
      preventClick: moved,
    };
  };

  const addPointFromCanvas = (event) => {
    if (!canvasRef.current) return;

    if (dragStateRef.current.preventClick) {
      dragStateRef.current.preventClick = false;
      return;
    }

    const canvasPoint = getCanvasEventPosition(canvasRef.current, event);
    const point = canvasToWorld(canvasPoint.x, canvasPoint.y);

    if (!isInsideMap(point)) {
      setStatus("РљР»РёРєРЅРёС‚Рµ РІРЅСѓС‚СЂРё СЃС‚Р°СЂРѕР№ РєР°СЂС‚С‹.");
      return;
    }

    if (activePointKind === "limit" && plannerModel.activeZone?.closed) {
      setStatus(
        "Р—РѕРЅР° СѓР¶Рµ Р·Р°РјРєРЅСѓС‚Р°. РќР°Р¶РјРёС‚Рµ В«РћС‚РєСЂС‹С‚СЊВ», С‡С‚РѕР±С‹ РґРѕР±Р°РІРёС‚СЊ РёР»Рё РїРѕРїСЂР°РІРёС‚СЊ С‚РѕС‡РєРё."
      );
      return;
    }

    setPoints((prev) => [
      ...prev,
      {
        ...point,
        kind: activePointKind,
        zoneId: activePointKind === "limit" ? activeLimitZoneId : null,
        task: activePointKind === "visit" ? DEFAULT_POINT_TASK : null,
      },
    ]);
    if (activePointKind === "visit") clearRouteState();
    else clearRouteState({ dropSolvedRoute: false });
    setStatus(
      activePointKind === "visit"
        ? "Р”РѕР±Р°РІР»РµРЅР° С‚РѕС‡РєР° РґР»СЏ РїРѕСЃРµС‰РµРЅРёСЏ."
        : `Р”РѕР±Р°РІР»РµРЅР° С‚РѕС‡РєР° РІ ${plannerModel.activeZoneName}.`
    );
  };

  const clearPoints = (kind = null) => {
    if (kind === "limit") resetZones();
    setPoints((prev) => (kind ? prev.filter((point) => point.kind !== kind) : []));
    if (kind === "limit") clearRouteState({ dropSolvedRoute: false });
    else clearRouteState();
    setStatus(
      kind === "visit"
        ? "РўРѕС‡РєРё РјР°СЂС€СЂСѓС‚Р° РѕС‡РёС‰РµРЅС‹."
        : kind === "limit"
          ? "Р’СЃРµ РѕРіСЂР°РЅРёС‡РёРІР°СЋС‰РёРµ Р·РѕРЅС‹ РѕС‡РёС‰РµРЅС‹."
          : "Р’СЃРµ С‚РѕС‡РєРё РѕС‡РёС‰РµРЅС‹."
    );
  };

  const deletePoint = (index) => {
    const targetPoint = points[index];
    setPoints((prev) => prev.filter((_, pointIndex) => pointIndex !== index));
    if (targetPoint?.kind === "limit") clearRouteState({ dropSolvedRoute: false });
    else clearRouteState();
  };

  const updatePointTask = (index, task) => {
    setPoints((prev) =>
      prev.map((point, pointIndex) => (pointIndex === index ? { ...point, task } : point))
    );
    clearRouteState();
  };

  const handleRouteTaskChange = (nextTaskKey) => {
    setRouteTaskKey(nextTaskKey);
    clearRouteState();
    setStatus("");
  };

  const handleAlgorithmChange = (nextAlgorithmKey) => {
    setAlgorithmKey(nextAlgorithmKey);
    clearRouteState();
    setStatus("");
  };

  const optimizeRoute = async () => {
    if (isOptimizing) return;

    if (plannerModel.visitPoints.length < 2) {
      setStatus("Р”РѕР±Р°РІСЊС‚Рµ С…РѕС‚СЏ Р±С‹ РґРІРµ С‚РѕС‡РєРё РґР»СЏ РїРѕСЃРµС‰РµРЅРёСЏ.");
      return;
    }

    setIsOptimizing(true);
    setStatus("РЎС‚СЂРѕРёРј РјР°СЂС€СЂСѓС‚...");

    try {
      const routeAnchor = getRouteAnchor(telemetry);
      const solveResult = await solveRouteWithNativeAlgorithm(
        plannerModel.visitPoints,
        algorithmKey,
        selectedAlgorithmParams,
        routeTaskKey
      );
      let solvedRoute = solveResult.route;

      if (routeTaskKey === "tsp" && solvedRoute.length) {
        solvedRoute = rotateClosedRouteToNearestPoint(solvedRoute, routeAnchor);
      }

      const routed =
        buildObstacleAwareRoute(solvedRoute, plannerModel.previewPolygons) ||
        solvedRoute.map((point) => ({ ...point }));
      const blocked = routeCrossesAnyLimitPolygon(routed, plannerModel.previewPolygons);
      setRouteSeed(solvedRoute);
      setOptimizedRoute(routed);
      setStatus(
        blocked
          ? "РњР°СЂС€СЂСѓС‚ РїРѕСЃС‚СЂРѕРµРЅ, РЅРѕ РІСЃРµ РµС‰Рµ РїРµСЂРµСЃРµРєР°РµС‚ РѕРіСЂР°РЅРёС‡РёРІР°СЋС‰РёР№ РєРѕРЅС‚СѓСЂ."
          : plannerModel.adjustedVisits.length
            ? `РњР°СЂС€СЂСѓС‚ РїРѕСЃС‚СЂРѕРµРЅ: ${getTaskLabel(routeTaskKey)} (${getAlgorithmLabel(algorithmKey)}). ${plannerModel.adjustedVisits.length} С‚РѕС‡РµРє Р°РІС‚РѕРјР°С‚РёС‡РµСЃРєРё СЃРґРІРёРЅСѓС‚С‹ Рє Р±Р»РёР¶Р°Р№С€РµР№ Р±РµР·РѕРїР°СЃРЅРѕР№ РїРѕР·РёС†РёРё.`
            : `РњР°СЂС€СЂСѓС‚ РїРѕСЃС‚СЂРѕРµРЅ: ${getTaskLabel(routeTaskKey)} (${getAlgorithmLabel(algorithmKey)}).`
      );
    } catch (error) {
      setRouteSeed([]);
      setOptimizedRoute([]);
      setStatus(
        error instanceof Error ? error.message : "РќРµ СѓРґР°Р»РѕСЃСЊ РїРѕСЃС‚СЂРѕРёС‚СЊ РјР°СЂС€СЂСѓС‚."
      );
    } finally {
      setIsOptimizing(false);
    }
  };

  const sendRoute = () => {
    if (!optimizedRoute.length) {
      setStatus("РЎРЅР°С‡Р°Р»Р° РїРѕСЃС‚СЂРѕР№С‚Рµ РјР°СЂС€СЂСѓС‚.");
      return;
    }

    if (plannerModel.routeBlocked) {
      setStatus("РњР°СЂС€СЂСѓС‚ РІСЃРµ РµС‰Рµ РїРµСЂРµСЃРµРєР°РµС‚ РѕРіСЂР°РЅРёС‡РёРІР°СЋС‰РёР№ РєРѕРЅС‚СѓСЂ.");
      return;
    }

    const controllerRouteSource =
      routeSeed.length > 1
        ? buildObstacleAwareRoute(routeSeed, plannerModel.polygons) || routeSeed
        : optimizedRoute;
    let routeForController = sanitizeRouteForController(controllerRouteSource);
    if (routeTaskKey === "tsp" && routeForController.length > 1) {
      routeForController = rotateClosedRouteToNearestPoint(
        routeForController,
        getRouteAnchor(telemetry)
      );
    }
    if (routeForController.length < 2) {
      setStatus("РњР°СЂС€СЂСѓС‚ СЃР»РёС€РєРѕРј РєРѕСЂРѕС‚РєРёР№ РїРѕСЃР»Рµ РѕС‡РёСЃС‚РєРё.");
      return;
    }

    const payload = {
      type: "route",
      algorithm: {
        key: algorithmKey,
        task: routeTaskKey,
        params: selectedAlgorithmParams,
      },
      route: routeForController.map((point) => ({ x: point.x, y: point.y })),
    };

    const sendPayload = (socket) => {
      socket.send(JSON.stringify(payload));
      setStatus(`РњР°СЂС€СЂСѓС‚ РѕС‚РїСЂР°РІР»РµРЅ (${routeForController.length} С‚РѕС‡РµРє).`);
    };

    const ws = routeWsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      const temp = new WebSocket(ROUTE_WS_URL);
      routeWsRef.current = temp;
      temp.onopen = () => {
        setRouteWsUp(true);
        sendPayload(temp);
      };
      temp.onclose = () => setRouteWsUp(false);
      temp.onerror = () => {
        setRouteWsUp(false);
        setStatus("РћС€РёР±РєР° СЃРѕРµРґРёРЅРµРЅРёСЏ СЃ РјР°СЂС€СЂСѓС‚РѕРј.");
      };
      return;
    }

    sendPayload(ws);
  };

  return (
    <div className="flex h-screen bg-stone-100 text-stone-900">
      <PlannerLeftSidebar
        activePointKind={activePointKind}
        onActivePointKindChange={setActivePointKind}
        onClearVisitPoints={() => clearPoints("visit")}
        onClearLimitPoints={() => clearPoints("limit")}
        routeTaskKey={routeTaskKey}
        onRouteTaskChange={handleRouteTaskChange}
        algorithmKey={algorithmKey}
        onAlgorithmChange={handleAlgorithmChange}
        status={status}
        routeBlocked={plannerModel.routeBlocked}
        algorithmFields={algorithmFields}
        selectedAlgorithmParams={selectedAlgorithmParams}
        onAlgorithmParamChange={updateAlgorithmParam}
        isOptimizing={isOptimizing}
        onOptimizeRoute={optimizeRoute}
        onSendRoute={sendRoute}
        onClearAll={() => clearPoints()}
        hasRoute={optimizedRoute.length > 0}
        routeLength={plannerModel.routeLength}
        visitCount={plannerModel.visitEntries.length}
        zoneCount={plannerModel.zoneEntries.length}
        polygonCount={plannerModel.polygons.length}
        adjustedVisitCount={plannerModel.adjustedVisits.length}
        activeZoneName={plannerModel.activeZoneName}
      />

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

      <PlannerRightSidebar
        activeZone={plannerModel.activeZone}
        activeZoneName={plannerModel.activeZoneName}
        activeLimitZoneId={activeLimitZoneId}
        zoneEntries={plannerModel.zoneEntries}
        visitEntries={plannerModel.visitEntries}
        plannedVisitEntries={plannerModel.plannedVisitEntries}
        expandedPoint={expandedPoint}
        hoveredPointIndex={hoveredPointIndex}
        visitsInsideLimitCount={plannerModel.visitsInsideLimit.length}
        polygonCount={plannerModel.polygons.length}
        adjustedVisitCount={plannerModel.adjustedVisits.length}
        routeBlocked={plannerModel.routeBlocked}
        telemetry={telemetry}
        telemetryWsUp={telemetryWsUp}
        routeWsUp={routeWsUp}
        solverApiUp={solverApiUp}
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
    </div>
  );
}
