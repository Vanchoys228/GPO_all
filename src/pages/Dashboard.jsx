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
import { useMemo } from "react";
import {
  DEFAULT_POINT_TASK,
  buildObstacleAwareRoute,
  canvasToWorld,
  DEFAULT_SURFACE_ZONES,
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
import {
  DEFAULT_BATTERY_RANGE_METERS,
  planRouteWithCharging,
} from "../lib/chargingPlanner";
import { DEFAULT_ENERGY_OPTIONS } from "../lib/energyModel";
import PlannerCanvas from "../components/dashboard/PlannerCanvas";
import PlannerLeftSidebar from "../components/dashboard/PlannerLeftSidebar";
import PlannerRightSidebar from "../components/dashboard/PlannerRightSidebar";

const ENERGY_SHORTAGE_FALLBACK =
  "Запаса хода не хватает: добавьте станции зарядки или увеличьте запас.";

const getEnergyWarningText = (routeBuildResult) => {
  if (!routeBuildResult || routeBuildResult.ok) return "";
  if (routeBuildResult.reason === "insufficient_range") {
    return routeBuildResult.error || ENERGY_SHORTAGE_FALLBACK;
  }
  if (routeBuildResult.reason === "invalid_battery_range") {
    return routeBuildResult.error || "Проверьте корректность запаса хода.";
  }
  return "";
};

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

const buildRouteWithEnergyStops = ({
  seedRoute,
  polygons,
  surfaceZones,
  chargingStations,
  batteryRangeMeters,
  energyOptions,
}) => {
  const safeRoute = buildObstacleAwareRoute(seedRoute, polygons);
  if (!safeRoute) {
    return {
      ok: false,
      reason: "obstacle_routing_failed",
      error: "Не удалось безопасно провести маршрут через текущие ограничивающие зоны.",
    };
  }

  const chargingResult = planRouteWithCharging({
    route: safeRoute,
    stations: chargingStations,
    polygons,
    surfaceZones,
    energyOptions,
    batteryRange: batteryRangeMeters,
  });
  if (!chargingResult.ok) {
    return {
      ok: false,
      reason: chargingResult.reason || "charging_planning_failed",
      error: chargingResult.error || "Маршрут недостижим при текущем запасе хода.",
    };
  }

  return {
    ok: true,
    route: chargingResult.route,
    stationStopCount: chargingResult.stationStopCount || 0,
    routeDistance: chargingResult.routeDistance,
    routeEnergy: chargingResult.routeEnergy || 0,
    estimatedTimeSec: chargingResult.estimatedTimeSec || 0,
    limitingMaxSpeedMps: chargingResult.limitingMaxSpeedMps || energyOptions?.speedMps || 0,
    averageSlipRisk: chargingResult.averageSlipRisk || 0,
  };
};

const parseLooseNumber = (rawValue) => {
  const normalized = String(rawValue ?? "")
    .trim()
    .replace(",", ".");
  if (!normalized) return Number.NaN;
  return Number(normalized);
};

const formatNumber = (value, digits) =>
  Number(value.toFixed(digits)).toString();

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
  const [energyWarning, setEnergyWarning] = useState("");
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
  const [batteryRangeMeters, setBatteryRangeMeters] = useState(
    DEFAULT_BATTERY_RANGE_METERS
  );
  const [cruiseSpeedMps, setCruiseSpeedMps] = useState(
    DEFAULT_ENERGY_OPTIONS.speedMps
  );
  const [payloadKg, setPayloadKg] = useState(DEFAULT_ENERGY_OPTIONS.payloadKg);
  const [batteryRangeInput, setBatteryRangeInput] = useState(
    String(DEFAULT_BATTERY_RANGE_METERS)
  );
  const [cruiseSpeedInput, setCruiseSpeedInput] = useState(
    formatNumber(DEFAULT_ENERGY_OPTIONS.speedMps, 3)
  );
  const [payloadInput, setPayloadInput] = useState(
    formatNumber(DEFAULT_ENERGY_OPTIONS.payloadKg, 2)
  );
  const [routeEnergyStats, setRouteEnergyStats] = useState({
    routeEnergy: 0,
    estimatedTimeSec: 0,
    limitingMaxSpeedMps: DEFAULT_ENERGY_OPTIONS.speedMps,
    averageSlipRisk: 0,
  });
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
    surfaceZones: DEFAULT_SURFACE_ZONES,
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
  const chargePointsRoutingText = JSON.stringify(
    plannerModel.chargePoints.map((point) => ({
      x: Number(point.x.toFixed(4)),
      y: Number(point.y.toFixed(4)),
    }))
  );
  const energyOptions = useMemo(
    () => ({
      speedMps: cruiseSpeedMps,
      payloadKg,
    }),
    [cruiseSpeedMps, payloadKg]
  );
  const autoRouteSyncToken = `${zoneSyncPayloadText}|${chargePointsRoutingText}|${batteryRangeMeters}|${cruiseSpeedMps}|${payloadKg}`;

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
      setEnergyWarning("");
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
      return;
    }

    const previewPolygons = JSON.parse(previewPolygonRoutingText);
    const chargingStations = JSON.parse(chargePointsRoutingText);
    const nextRoute = buildRouteWithEnergyStops({
      seedRoute: routeSeed,
      polygons: previewPolygons,
      surfaceZones: plannerModel.surfaceZones,
      chargingStations,
      batteryRangeMeters,
      energyOptions,
    });
    if (!nextRoute.ok) {
      setOptimizedRoute([]);
      setEnergyWarning(getEnergyWarningText(nextRoute));
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
      setStatus(nextRoute.error || "Маршрут недостижим при текущих ограничениях.");
      return;
    }
    setEnergyWarning("");
    setOptimizedRoute(nextRoute.route);
    setRouteEnergyStats({
      routeEnergy: nextRoute.routeEnergy,
      estimatedTimeSec: nextRoute.estimatedTimeSec,
      limitingMaxSpeedMps: nextRoute.limitingMaxSpeedMps,
      averageSlipRisk: nextRoute.averageSlipRisk,
    });
  }, [
    batteryRangeMeters,
    chargePointsRoutingText,
    energyOptions,
    plannerModel.surfaceZones,
    previewPolygonRoutingText,
    routeSeed,
  ]);

  useEffect(() => {
    if (lastAutoRouteZoneSyncRef.current === autoRouteSyncToken) return;
    lastAutoRouteZoneSyncRef.current = autoRouteSyncToken;
    if (routeSeed.length < 2) {
      setEnergyWarning("");
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
      return;
    }

    const controllerPolygonsPayload = JSON.parse(zoneSyncPayloadText);
    const controllerPolygons = (controllerPolygonsPayload?.zones || []).map((zone) => ({
      id: zone.id,
      name: zone.name,
      points: Array.isArray(zone.points) ? zone.points : [],
    }));
    const chargingStations = JSON.parse(chargePointsRoutingText);

    const rebuilt = buildRouteWithEnergyStops({
      seedRoute: routeSeed,
      polygons: controllerPolygons,
      surfaceZones: plannerModel.surfaceZones,
      chargingStations,
      batteryRangeMeters,
      energyOptions,
    });
    if (!rebuilt.ok) {
      setEnergyWarning(getEnergyWarningText(rebuilt));
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
      setStatus(rebuilt.error || "Невозможно безопасно перестроить маршрут.");
      return;
    }
    setEnergyWarning("");
    setRouteEnergyStats({
      routeEnergy: rebuilt.routeEnergy,
      estimatedTimeSec: rebuilt.estimatedTimeSec,
      limitingMaxSpeedMps: rebuilt.limitingMaxSpeedMps,
      averageSlipRisk: rebuilt.averageSlipRisk,
    });
    const routeForController = sanitizeRouteForController(rebuilt.route);
    if (routeForController.length < 2) {
      setStatus("Маршрут стал слишком коротким после перестройки под зоны.");
      return;
    }

    const payload = {
      type: "route",
      algorithm: {
        key: algorithmKey,
        task: routeTaskKey,
        params: selectedAlgorithmParams,
      },
      motion: {
        cruiseSpeedMps,
        payloadKg,
        batteryRange: batteryRangeMeters,
      },
      route: routeForController.map((point) => ({ x: point.x, y: point.y })),
    };

    sendRouteChannelPayload(routeWsRef, payload, {
      onSent: () => {
        const chargingSuffix = rebuilt.stationStopCount
          ? `, зарядок: ${rebuilt.stationStopCount}`
          : "";
        setStatus(
          `Маршрут обновлён после изменения ограничивающих зон (${routeForController.length} точек${chargingSuffix}).`
        );
      },
    });
  }, [
    algorithmKey,
    autoRouteSyncToken,
    batteryRangeMeters,
    chargePointsRoutingText,
    cruiseSpeedMps,
    energyOptions,
    payloadKg,
    plannerModel.surfaceZones,
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
      setEnergyWarning("");
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
    }
  };

  const createZone = () => {
    const zone = {
      id: `zone-${nextZoneNumber}`,
      name: `Зона ${nextZoneNumber}`,
      closed: false,
    };
    setLimitZones((prev) => [...prev, zone]);
    setActiveLimitZoneId(zone.id);
    setNextZoneNumber((prev) => prev + 1);
    setActivePointKind("limit");
    setStatus(`Создана ${zone.name}.`);
  };

  const selectZone = (zoneId) => {
    setActiveLimitZoneId(zoneId);
    setActivePointKind("limit");
  };

  const toggleZoneClosed = (zoneId) => {
    const target = plannerModel.zoneEntries.find((zone) => zone.id === zoneId);
    if (!target) return;

    if (!target.closed && target.points.length < 3) {
      setStatus("Чтобы замкнуть зону, нужно минимум три точки.");
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
        ? `${target.name} открыта для редактирования.`
        : `${target.name} замкнута.`
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
    setStatus("Точки выбранной зоны очищены.");
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
    setStatus("Ограничивающая зона удалена.");
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
      setStatus("Кликните внутри рабочей карты.");
      return;
    }

    if (activePointKind === "limit" && plannerModel.activeZone?.closed) {
      setStatus(
        "Зона уже замкнута. Нажмите «Открыть», чтобы добавлять или менять точки."
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
        ? "Добавлена точка посещения."
        : activePointKind === "charge"
          ? "Добавлена станция зарядки."
          : `Добавлена точка в ${plannerModel.activeZoneName}.`
    );
  };

  const clearPoints = (kind = null) => {
    if (kind === "limit") resetZones();
    setPoints((prev) => (kind ? prev.filter((point) => point.kind !== kind) : []));
    if (kind === "visit") clearRouteState();
    else if (kind === "limit" || kind === "charge") {
      clearRouteState({ dropSolvedRoute: false });
    } else {
      clearRouteState();
    }
    setStatus(
      kind === "visit"
        ? "Маршрутные точки очищены."
        : kind === "charge"
          ? "Станции зарядки очищены."
        : kind === "limit"
          ? "Ограничивающие зоны очищены."
          : "Все точки очищены."
    );
  };

  const deletePoint = (index) => {
    const targetPoint = points[index];
    setPoints((prev) => prev.filter((_, pointIndex) => pointIndex !== index));
    if (targetPoint?.kind === "visit") clearRouteState();
    else clearRouteState({ dropSolvedRoute: false });
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
    setEnergyWarning("");
  };

  const handleAlgorithmChange = (nextAlgorithmKey) => {
    setAlgorithmKey(nextAlgorithmKey);
    clearRouteState();
    setStatus("");
    setEnergyWarning("");
  };

  const invalidateEnergyDependentRoute = () => {
    clearRouteState({ dropSolvedRoute: false });
    setRouteEnergyStats((prev) => ({
      ...prev,
      routeEnergy: 0,
      estimatedTimeSec: 0,
      averageSlipRisk: 0,
    }));
    setEnergyWarning("");
  };

  const handleBatteryRangeChange = (rawValue) => {
    setBatteryRangeInput(rawValue);
    const parsed = parseLooseNumber(rawValue);
    if (!Number.isFinite(parsed)) return;
    const nextValue = Math.max(1, Math.min(10000, Math.round(parsed)));
    if (nextValue === batteryRangeMeters) return;
    setBatteryRangeMeters(nextValue);
    invalidateEnergyDependentRoute();
  };

  const handleBatteryRangeBlur = () => {
    const parsed = parseLooseNumber(batteryRangeInput);
    if (!Number.isFinite(parsed)) {
      setBatteryRangeInput(String(batteryRangeMeters));
      return;
    }
    const nextValue = Math.max(1, Math.min(10000, Math.round(parsed)));
    if (nextValue !== batteryRangeMeters) {
      setBatteryRangeMeters(nextValue);
      invalidateEnergyDependentRoute();
    }
    setBatteryRangeInput(String(nextValue));
  };

  const handleCruiseSpeedChange = (rawValue) => {
    setCruiseSpeedInput(rawValue);
    const parsed = parseLooseNumber(rawValue);
    if (!Number.isFinite(parsed)) return;
    const nextValue = Math.max(0.05, Math.min(0.8, Number(parsed.toFixed(3))));
    if (Math.abs(nextValue - cruiseSpeedMps) <= 1e-9) return;
    setCruiseSpeedMps(nextValue);
    invalidateEnergyDependentRoute();
  };

  const handleCruiseSpeedBlur = () => {
    const parsed = parseLooseNumber(cruiseSpeedInput);
    if (!Number.isFinite(parsed)) {
      setCruiseSpeedInput(formatNumber(cruiseSpeedMps, 3));
      return;
    }
    const nextValue = Math.max(0.05, Math.min(0.8, Number(parsed.toFixed(3))));
    if (Math.abs(nextValue - cruiseSpeedMps) > 1e-9) {
      setCruiseSpeedMps(nextValue);
      invalidateEnergyDependentRoute();
    }
    setCruiseSpeedInput(formatNumber(nextValue, 3));
  };

  const handlePayloadChange = (rawValue) => {
    setPayloadInput(rawValue);
    const parsed = parseLooseNumber(rawValue);
    if (!Number.isFinite(parsed)) return;
    const nextValue = Math.max(0, Math.min(500, Number(parsed.toFixed(2))));
    if (Math.abs(nextValue - payloadKg) <= 1e-9) return;
    setPayloadKg(nextValue);
    invalidateEnergyDependentRoute();
  };

  const handlePayloadBlur = () => {
    const parsed = parseLooseNumber(payloadInput);
    if (!Number.isFinite(parsed)) {
      setPayloadInput(formatNumber(payloadKg, 2));
      return;
    }
    const nextValue = Math.max(0, Math.min(500, Number(parsed.toFixed(2))));
    if (Math.abs(nextValue - payloadKg) > 1e-9) {
      setPayloadKg(nextValue);
      invalidateEnergyDependentRoute();
    }
    setPayloadInput(formatNumber(nextValue, 2));
  };

  const optimizeRoute = async () => {
    if (isOptimizing) return;

    if (plannerModel.visitPoints.length < 2) {
      setStatus("Добавьте хотя бы две точки посещения.");
      setEnergyWarning("");
      return;
    }

    setIsOptimizing(true);
    setStatus("Строим маршрут...");

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

      const routed = buildRouteWithEnergyStops({
        seedRoute: solvedRoute,
        polygons: plannerModel.previewPolygons,
        surfaceZones: plannerModel.surfaceZones,
        chargingStations: plannerModel.chargePoints,
        batteryRangeMeters,
        energyOptions,
      });
      if (!routed.ok) {
        setRouteSeed(solvedRoute);
        setOptimizedRoute([]);
        setRouteEnergyStats((prev) => ({
          ...prev,
          routeEnergy: 0,
          estimatedTimeSec: 0,
          averageSlipRisk: 0,
        }));
        setEnergyWarning(getEnergyWarningText(routed));
        setStatus(routed.error || "Не удалось построить достижимый маршрут.");
        return;
      }
      const blocked = routeCrossesAnyLimitPolygon(
        routed.route,
        plannerModel.previewPolygons
      );
      setRouteSeed(solvedRoute);
      setOptimizedRoute(routed.route);
      setRouteEnergyStats({
        routeEnergy: routed.routeEnergy,
        estimatedTimeSec: routed.estimatedTimeSec,
        limitingMaxSpeedMps: routed.limitingMaxSpeedMps,
        averageSlipRisk: routed.averageSlipRisk,
      });
      setEnergyWarning("");
      const chargingSuffix = routed.stationStopCount
        ? ` Добавлено заездов на зарядку: ${routed.stationStopCount}.`
        : "";
      const energySuffix = ` Энергия: ${routed.routeEnergy.toFixed(1)} ед., время: ${routed.estimatedTimeSec.toFixed(1)} с.`;
      setStatus(
        blocked
          ? "Маршрут построен, но всё ещё пересекает ограничивающий контур."
          : plannerModel.adjustedVisits.length
            ? `Маршрут построен: ${getTaskLabel(routeTaskKey)} (${getAlgorithmLabel(algorithmKey)}). ${plannerModel.adjustedVisits.length} точек автоматически сдвинуты к безопасной позиции.${chargingSuffix}${energySuffix}`
            : `Маршрут построен: ${getTaskLabel(routeTaskKey)} (${getAlgorithmLabel(algorithmKey)}).${chargingSuffix}${energySuffix}`
      );
    } catch (error) {
      setRouteSeed([]);
      setOptimizedRoute([]);
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
      setEnergyWarning("");
      setStatus(
        error instanceof Error ? error.message : "Не удалось построить маршрут."
      );
    } finally {
      setIsOptimizing(false);
    }
  };

  const sendRoute = () => {
    if (!optimizedRoute.length) {
      setStatus("Сначала постройте маршрут.");
      return;
    }

    if (plannerModel.routeBlocked) {
      setStatus("Маршрут всё ещё пересекает ограничивающий контур.");
      return;
    }

    let controllerRouteSource = optimizedRoute;
    let chargingStops = 0;
    if (routeSeed.length > 1) {
      const rebuiltForController = buildRouteWithEnergyStops({
        seedRoute: routeSeed,
        polygons: plannerModel.polygons,
        surfaceZones: plannerModel.surfaceZones,
        chargingStations: plannerModel.chargePoints,
        batteryRangeMeters,
        energyOptions,
      });
      if (!rebuiltForController.ok) {
        setEnergyWarning(getEnergyWarningText(rebuiltForController));
        setStatus(
          rebuiltForController.error ||
            "Невозможно безопасно построить маршрут через текущие зоны."
        );
        return;
      }
      controllerRouteSource = rebuiltForController.route;
      chargingStops = rebuiltForController.stationStopCount;
    }
    const routeForController = sanitizeRouteForController(controllerRouteSource);
    if (routeForController.length < 2) {
      setStatus("Маршрут слишком короткий после очистки.");
      return;
    }

    const payload = {
      type: "route",
      algorithm: {
        key: algorithmKey,
        task: routeTaskKey,
        params: selectedAlgorithmParams,
      },
      motion: {
        cruiseSpeedMps,
        payloadKg,
        batteryRange: batteryRangeMeters,
      },
      route: routeForController.map((point) => ({ x: point.x, y: point.y })),
    };

    const sendPayload = (socket) => {
      socket.send(JSON.stringify(payload));
      const chargingSuffix = chargingStops ? `, зарядок: ${chargingStops}` : "";
      setEnergyWarning("");
      setStatus(`Маршрут отправлен (${routeForController.length} точек${chargingSuffix}).`);
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
        setStatus("Ошибка соединения с маршрутом.");
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
        chargeEntries={plannerModel.chargeEntries}
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
