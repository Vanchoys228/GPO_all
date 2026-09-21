import { useEffect, useRef, useState } from "react";
import {
  ALGORITHM_OPTIONS,
  TASK_OPTIONS,
  getAlgorithmFields,
  getDefaultAlgorithmParams,
  probeNativeSolver,
  solveRouteWithNativeAlgorithm,
} from "../lib/routeAlgorithms";
import { useMemo } from "react";
import {
  CANVAS_HEIGHT,
  CANVAS_WIDTH,
  DEFAULT_POINT_TASK,
  HALF_HEIGHT,
  HALF_WIDTH,
  SCALE,
  buildObstacleAwareRoute,
  canvasToWorld,
  DEFAULT_SURFACE_ZONES,
  drawPlannerBackground,
  isInsideMap,
  pointInAnyPolygon,
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
import {
  ArrowLeft,
  PanelLeftClose,
  PanelLeftOpen,
  BatteryCharging,
  Bot,
  Map as MapIcon,
  Route,
  Shield,
  SlidersHorizontal,
} from "lucide-react";

const ENERGY_SHORTAGE_FALLBACK =
  "Запаса хода не хватает: добавьте станции зарядки или увеличьте запас.";

const MAPPING_SURVEY_MODES = [
  { key: "snake", label: "Змейка" },
  { key: "double", label: "Двойной объезд" },
];

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

const randomBetween = (min, max) => min + Math.random() * (max - min);

const isFinitePoint = (point) =>
  Number.isFinite(point?.x) && Number.isFinite(point?.y);

const normalizeImportedZoneMeta = (zone, index) => ({
  id:
    typeof zone?.id === "string" && zone.id.trim()
      ? zone.id.trim()
      : `zone-${index + 1}`,
  name:
    typeof zone?.name === "string" && zone.name.trim()
      ? zone.name.trim()
      : `Зона ${index + 1}`,
  closed: Boolean(zone?.closed),
});

const deriveNextZoneNumber = (zones) => {
  const maxNumber = zones.reduce((best, zone) => {
    const idMatch = String(zone?.id ?? "").match(/zone-(\d+)/i);
    const nameMatch = String(zone?.name ?? "").match(/(\d+)/);
    const candidates = [idMatch?.[1], nameMatch?.[1]]
      .map((value) => Number(value))
      .filter(Number.isFinite);

    return candidates.length ? Math.max(best, ...candidates) : best;
  }, 1);

  return Math.max(2, maxNumber + 1);
};

const normalizeImportedGraph = (rawGraph) => {
  if (!rawGraph || typeof rawGraph !== "object") {
    throw new Error("Граф должен быть объектом JSON.");
  }

  if (Array.isArray(rawGraph.points)) {
    const zonesSource =
      Array.isArray(rawGraph.limitZones) && rawGraph.limitZones.length
        ? rawGraph.limitZones
        : [INITIAL_ZONE];
    const limitZones = zonesSource.map((zone, index) =>
      normalizeImportedZoneMeta(zone, index)
    );
    const zoneIds = new Set(limitZones.map((zone) => zone.id));
    const points = rawGraph.points
      .filter((point) => isFinitePoint(point))
      .filter((point) => isInsideMap(point))
      .map((point) => {
        if (point.kind === "limit") {
          return {
            x: point.x,
            y: point.y,
            kind: "limit",
            zoneId: zoneIds.has(point.zoneId) ? point.zoneId : limitZones[0].id,
            task: null,
          };
        }

        if (point.kind === "charge") {
          return {
            x: point.x,
            y: point.y,
            kind: "charge",
            zoneId: null,
            task: null,
          };
        }

        return {
          x: point.x,
          y: point.y,
          kind: "visit",
          zoneId: null,
          task: point.task || DEFAULT_POINT_TASK,
        };
      });

    return {
      points,
      limitZones,
      routeTaskKey: rawGraph.routeTaskKey,
      algorithmKey: rawGraph.algorithmKey,
      activeLimitZoneId: rawGraph.activeLimitZoneId,
    };
  }

  const limitZones = Array.isArray(rawGraph.zoneEntries)
    ? rawGraph.zoneEntries.map((zone, index) => normalizeImportedZoneMeta(zone, index))
    : [INITIAL_ZONE];
  const points = [];

  for (const visitEntry of Array.isArray(rawGraph.visitEntries) ? rawGraph.visitEntries : []) {
    const point = visitEntry?.point;
    if (!isFinitePoint(point) || !isInsideMap(point)) continue;
    points.push({
      x: point.x,
      y: point.y,
      kind: "visit",
      zoneId: null,
      task: visitEntry?.task || point?.task || DEFAULT_POINT_TASK,
    });
  }

  for (const chargeEntry of Array.isArray(rawGraph.chargeEntries) ? rawGraph.chargeEntries : []) {
    const point = chargeEntry?.point;
    if (!isFinitePoint(point) || !isInsideMap(point)) continue;
    points.push({
      x: point.x,
      y: point.y,
      kind: "charge",
      zoneId: null,
      task: null,
    });
  }

  limitZones.forEach((zone) => {
    const sourceZone = Array.isArray(rawGraph.zoneEntries)
      ? rawGraph.zoneEntries.find((entry) => String(entry?.id) === zone.id)
      : null;
    const sourcePoints = Array.isArray(sourceZone?.points) ? sourceZone.points.slice() : [];
    sourcePoints
      .sort((left, right) => (Number(left?.order) || 0) - (Number(right?.order) || 0))
      .forEach((entry) => {
        const point = entry?.point;
        if (!isFinitePoint(point) || !isInsideMap(point)) return;
        points.push({
          x: point.x,
          y: point.y,
          kind: "limit",
          zoneId: zone.id,
          task: null,
        });
      });
  });

  return {
    points,
    limitZones,
    routeTaskKey: rawGraph.routeTaskKey,
    algorithmKey: rawGraph.algorithmKey,
    activeLimitZoneId: rawGraph.activeLimitZoneId,
  };
};

const pickRandomObstacleCenter = ({
  telemetry,
  optimizedRoute,
  points,
  polygons,
  obstacle,
}) => {
  const allPoints = Array.isArray(points) ? points : [];
  const route = Array.isArray(optimizedRoute) ? optimizedRoute : [];
  const obstacleSizeX = Number(obstacle?.sizeX) || 0.8;
  const obstacleSizeY = Number(obstacle?.sizeY) || 0.8;
  const obstacleRadius = Math.hypot(obstacleSizeX, obstacleSizeY) * 0.5;
  const protectedPointRadius = obstacleRadius + 0.55;
  const routeBiasAttempts = 28;
  const totalAttempts = 120;

  const isSafe = (candidate) => {
    if (!isInsideMap(candidate)) return false;
    if (pointInAnyPolygon(candidate, polygons)) return false;

    const robotDistance = Math.hypot(candidate.x - telemetry.x, candidate.y - telemetry.y);
    if (robotDistance < 1.1) return false;

    for (const point of allPoints) {
      if (Math.hypot(candidate.x - point.x, candidate.y - point.y) < protectedPointRadius) {
        return false;
      }
    }

    for (const point of route) {
      if (Math.hypot(candidate.x - point.x, candidate.y - point.y) < protectedPointRadius) {
        return false;
      }
    }

    return true;
  };

  for (let attempt = 0; attempt < totalAttempts; attempt += 1) {
    let candidate = null;
    const useRouteBias = route.length > 1 && attempt < routeBiasAttempts;

    if (useRouteBias) {
      const segmentIndex = Math.floor(Math.random() * (route.length - 1));
      const a = route[segmentIndex];
      const b = route[segmentIndex + 1];
      const t = Math.random();
      const ax = a.x + (b.x - a.x) * t;
      const ay = a.y + (b.y - a.y) * t;
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const segmentLength = Math.hypot(dx, dy);

      if (segmentLength > 1e-6) {
        const normalX = -dy / segmentLength;
        const normalY = dx / segmentLength;
        const sign = Math.random() < 0.5 ? -1 : 1;
        const lateralOffset = randomBetween(0.22, 0.85);
        candidate = {
          x: ax + normalX * lateralOffset * sign,
          y: ay + normalY * lateralOffset * sign,
        };
      }
    }

    if (!candidate) {
      candidate = {
        x: randomBetween(-HALF_WIDTH + 1.2, HALF_WIDTH - 1.2),
        y: randomBetween(-HALF_HEIGHT + 1.2, HALF_HEIGHT - 1.2),
      };
    }

    if (isSafe(candidate)) return candidate;
  }

  return null;
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
  const [notification, setNotification] = useState(null);
  const notificationTimerRef = useRef(null);
  const [expandedPoint, setExpandedPoint] = useState(null);
  const [hoveredPointIndex, setHoveredPointIndex] = useState(null);
  const [, setTelemetryWsUp] = useState(false);
  const [, setRouteWsUp] = useState(false);
  const [, setSolverApiUp] = useState(false);
  const [isOptimizing, setIsOptimizing] = useState(false);
  const [telemetry, setTelemetry] = useState(INITIAL_TELEMETRY);
  const [manualObstacles, setManualObstacles] = useState([]);
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
  const [mappingSurveyMode, setMappingSurveyMode] = useState(
    MAPPING_SURVEY_MODES[0].key
  );
  const [workspaceSection, setWorkspaceSection] = useState(null);
  const [sidebarOpen, setSidebarOpen] = useState(true);
  const [visibleLayers, setVisibleLayers] = useState({
    surfaces: true,
    zones: true,
    obstacleTrace: true,
    route: true,
  });
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

  const handleImportGraph = (rawGraph) => {
    const imported = normalizeImportedGraph(rawGraph);
    const importedZones =
      imported.limitZones.length > 0 ? imported.limitZones : [INITIAL_ZONE];

    setPoints(imported.points);
    setLimitZones(importedZones);
    setActiveLimitZoneId(
      importedZones.some((zone) => zone.id === imported.activeLimitZoneId)
        ? imported.activeLimitZoneId
        : importedZones[0].id
    );
    setNextZoneNumber(deriveNextZoneNumber(importedZones));
    setActivePointKind("visit");
    setExpandedPoint(null);
    setHoveredPointIndex(null);
    setRouteSeed([]);
    setOptimizedRoute([]);
    setRouteEnergyStats((prev) => ({
      ...prev,
      routeEnergy: 0,
      estimatedTimeSec: 0,
      averageSlipRisk: 0,
    }));

    if (typeof imported.routeTaskKey === "string") {
      const hasTask = TASK_OPTIONS.some((task) => task.key === imported.routeTaskKey);
      if (hasTask) setRouteTaskKey(imported.routeTaskKey);
    }

    if (typeof imported.algorithmKey === "string") {
      const hasAlgorithm = ALGORITHM_OPTIONS.some(
        (algorithm) => algorithm.key === imported.algorithmKey
      );
      if (hasAlgorithm) setAlgorithmKey(imported.algorithmKey);
    }

  };

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
      showNotification(getEnergyWarningText(nextRoute) || nextRoute.error || "Маршрут недостижим при текущих ограничениях.");
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
      showNotification(nextRoute.error || "Маршрут недостижим при текущих ограничениях.");
      return;
    }
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
      showNotification(getEnergyWarningText(rebuilt) || rebuilt.error || "Невозможно безопасно перестроить маршрут.");
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
      showNotification(rebuilt.error || "Невозможно безопасно перестроить маршрут.");
      return;
    }
    setRouteEnergyStats({
      routeEnergy: rebuilt.routeEnergy,
      estimatedTimeSec: rebuilt.estimatedTimeSec,
      limitingMaxSpeedMps: rebuilt.limitingMaxSpeedMps,
      averageSlipRisk: rebuilt.averageSlipRisk,
    });
    const routeForController = sanitizeRouteForController(rebuilt.route);
    if (routeForController.length < 2) {
      showNotification("Маршрут стал слишком коротким после перестройки под зоны.");
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

    sendRouteChannelPayload(routeWsRef, payload);
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

  const showNotification = (message, tone = "error") => {
    if (!message) return;
    setNotification({ message, tone });
  };

  useEffect(() => {
    if (!notification) return undefined;
    if (notificationTimerRef.current) {
      window.clearTimeout(notificationTimerRef.current);
    }
    notificationTimerRef.current = window.setTimeout(() => {
      setNotification(null);
      notificationTimerRef.current = null;
    }, 4500);
    return () => {
      if (notificationTimerRef.current) {
        window.clearTimeout(notificationTimerRef.current);
        notificationTimerRef.current = null;
      }
    };
  }, [notification]);

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
  };

  const selectZone = (zoneId) => {
    setActiveLimitZoneId(zoneId);
    setActivePointKind("limit");
  };

  const toggleZoneClosed = (zoneId) => {
    const target = plannerModel.zoneEntries.find((zone) => zone.id === zoneId);
    if (!target) return;

    if (!target.closed && target.points.length < 3) {
      showNotification("Чтобы замкнуть зону, нужно минимум три точки.");
      return;
    }

    setLimitZones((prev) =>
      prev.map((zone) =>
        zone.id === zoneId ? { ...zone, closed: !zone.closed } : zone
      )
    );
    clearRouteState({ dropSolvedRoute: false });
  };

  const clearZone = (zoneId) => {
    setPoints((prev) =>
      prev.filter((point) => point.kind !== "limit" || point.zoneId !== zoneId)
    );
    setLimitZones((prev) =>
      prev.map((zone) => (zone.id === zoneId ? { ...zone, closed: false } : zone))
    );
    clearRouteState({ dropSolvedRoute: false });
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
      return;
    }

    if (activePointKind === "limit" && plannerModel.activeZone?.closed) {
      showNotification("Зона уже замкнута. Нажмите «Открыть», чтобы добавлять или менять точки.");
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
  };

  const clearPoints = (kind = null) => {
    if (kind === "limit") resetZones();
    setPoints((prev) => (kind ? prev.filter((point) => point.kind !== kind) : []));
    if (kind === "visit") clearRouteState();
    else if (kind === "limit" || kind === "charge") {
      clearRouteState({ dropSolvedRoute: false });
    } else {
      clearRouteState();
      setTelemetry((prev) => ({
        ...prev,
        obstacleTrace: [],
        obstacleMap: { ...INITIAL_TELEMETRY.obstacleMap, cells: [] },
      }));
      setManualObstacles([]);
    }
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
  };

  const handleAlgorithmChange = (nextAlgorithmKey) => {
    setAlgorithmKey(nextAlgorithmKey);
    clearRouteState();
  };

  const invalidateEnergyDependentRoute = () => {
    clearRouteState({ dropSolvedRoute: false });
    setRouteEnergyStats((prev) => ({
      ...prev,
      routeEnergy: 0,
      estimatedTimeSec: 0,
      averageSlipRisk: 0,
    }));
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
      showNotification("Добавьте хотя бы две точки посещения.");
        return;
    }

    setIsOptimizing(true);

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
        showNotification(getEnergyWarningText(routed) || routed.error || "Не удалось построить достижимый маршрут.");
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
      if (blocked) showNotification("Маршрут построен, но всё ещё пересекает ограничивающий контур.");
    } catch (error) {
      setRouteSeed([]);
      setOptimizedRoute([]);
      setRouteEnergyStats((prev) => ({
        ...prev,
        routeEnergy: 0,
        estimatedTimeSec: 0,
        averageSlipRisk: 0,
      }));
      showNotification(error instanceof Error ? error.message : "Не удалось построить маршрут.");
    } finally {
      setIsOptimizing(false);
    }
  };

  const sendRoute = () => {
    if (!optimizedRoute.length) {
      showNotification("Сначала постройте маршрут.");
      return;
    }

    if (plannerModel.routeBlocked) {
      showNotification("Маршрут всё ещё пересекает ограничивающий контур.");
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
        showNotification(getEnergyWarningText(rebuiltForController) || rebuiltForController.error || "Невозможно безопасно построить маршрут через текущие зоны.");
        return;
      }
      controllerRouteSource = rebuiltForController.route;
      chargingStops = rebuiltForController.stationStopCount;
    }
    const routeForController = sanitizeRouteForController(controllerRouteSource);
    if (routeForController.length < 2) {
      showNotification("Маршрут слишком короткий после очистки.");
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
        showNotification(`Маршрут отправлен (${routeForController.length} точек${chargingSuffix}).`, "success");
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
        showNotification("Ошибка соединения с маршрутом.");
      };
      return;
    }

    sendPayload(ws);
  };

  const addRandomObstacle = () => {
    const obstacle = {
      sizeX: Number(randomBetween(0.46, 1.15).toFixed(3)),
      sizeY: Number(randomBetween(0.38, 0.95).toFixed(3)),
      height: Number(randomBetween(0.32, 0.9).toFixed(3)),
    };
    const center = pickRandomObstacleCenter({
      telemetry,
      optimizedRoute,
      points,
      polygons: plannerModel.polygons,
      obstacle,
    });

    if (!center) {
      showNotification("Не удалось подобрать безопасное место для случайного препятствия.");
      return;
    }

    const payload = {
      type: "spawn_random_obstacle",
      commandId: Date.now(),
      obstacle: {
        x: Number(center.x.toFixed(4)),
        y: Number(center.y.toFixed(4)),
        ...obstacle,
      },
    };

    setManualObstacles((current) => [...current, payload.obstacle]);

    sendRouteChannelPayload(routeWsRef, payload, {
      onSent: () => {
      },
      onError: () => {
        showNotification("Не удалось отправить команду добавления препятствия.");
      },
    });
  };

  const clearObstacles = () => {
    if (!manualObstacles.length) {
      showNotification("Созданных препятствий сейчас нет.");
      return;
    }

    const payload = {
      type: "clear_runtime_obstacles",
      commandId: Date.now(),
    };

    sendRouteChannelPayload(routeWsRef, payload, {
      onSent: () => {
        setManualObstacles([]);
        showNotification("Созданные препятствия очищены.", "success");
      },
      onError: () => {
        showNotification("Не удалось отправить команду очистки препятствий.");
      },
    });
  };

  const startMappingSurvey = () => {
    const payload = {
      type: "start_mapping_survey",
      commandId: Date.now(),
      clearMap: true,
      mode: mappingSurveyMode,
    };
    sendRouteChannelPayload(routeWsRef, payload, {
      onSent: () => {
      },
      onError: () => {
        showNotification("Не удалось отправить команду объезда карты.");
      },
    });
  };

  const exportMapImage = () => {
    if (!telemetry.obstacleMap?.cells?.length) {
      return;
    }

    const exportCanvas = document.createElement("canvas");
    exportCanvas.width = CANVAS_WIDTH;
    exportCanvas.height = CANVAS_HEIGHT;

    const ctx = exportCanvas.getContext("2d");
    if (!ctx) {
      return;
    }

    drawPlannerBackground(ctx, [], { annotate: false });

    const rawCellSize = Number(telemetry.obstacleMap.cellSize);
    const cellSize = Number.isFinite(rawCellSize) && rawCellSize > 0 ? rawCellSize : 0.06;
    const cellCanvasSize = Math.max(3, cellSize * SCALE * 0.92);

    telemetry.obstacleMap.cells.forEach((cell) => {
      const confidenceRaw = Number(cell?.confidence);
      const confidence = Number.isFinite(confidenceRaw) ? Math.max(0, confidenceRaw) : 0;
      const intensity = Math.max(0.16, Math.min(1, confidence / 6));
      const point = worldToCanvas(cell.x, cell.y);

      ctx.fillStyle = `rgba(14, 165, 233, ${0.12 + intensity * 0.3})`;
      ctx.strokeStyle = `rgba(2, 132, 199, ${0.18 + intensity * 0.38})`;
      ctx.lineWidth = 1;
      ctx.fillRect(
        point.x - cellCanvasSize / 2,
        point.y - cellCanvasSize / 2,
        cellCanvasSize,
        cellCanvasSize
      );
      ctx.strokeRect(
        point.x - cellCanvasSize / 2,
        point.y - cellCanvasSize / 2,
        cellCanvasSize,
        cellCanvasSize
      );
    });

    const link = document.createElement("a");
    const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
    const fileName = telemetry.obstacleMap?.imageFile || `obstacle-map-${timestamp}.png`;
    link.href = exportCanvas.toDataURL("image/png");
    link.download = fileName.endsWith(".png") ? fileName : `${fileName}.png`;
    link.click();
  };

  const workspaceSections = [
    { key: "route", label: "Маршрут", icon: Route },
    { key: "zones", label: "Зоны", icon: Shield },
    { key: "algorithm", label: "Алгоритм", icon: SlidersHorizontal },
    { key: "energy", label: "Энергия", icon: BatteryCharging },
    { key: "mapping", label: "Карта", icon: MapIcon },
  ];

  const showRouteSettings = ["route", "algorithm", "energy"].includes(workspaceSection);

  const toggleMapLayer = (layerKey) => {
    setVisibleLayers((current) => ({
      ...current,
      [layerKey]: !current[layerKey],
    }));
  };

  const selectWorkspaceSection = (sectionKey) => {
    setWorkspaceSection(sectionKey);
    if (sectionKey === "route") setActivePointKind("visit");
    if (sectionKey === "zones") setActivePointKind("limit");
    if (sectionKey === "energy") setActivePointKind("charge");
  };

  const returnToWorkspaceMenu = () => {
    setWorkspaceSection(null);
  };

  return (
    <div className="flex h-screen flex-col overflow-hidden bg-slate-100 text-slate-900">
      {notification && (
        <div className="pointer-events-none fixed right-4 top-4 z-[100] w-[min(420px,calc(100vw-2rem))]">
          <div
            role="alert"
            className={`rounded-lg border px-4 py-3 text-sm font-semibold shadow-lg ${
              notification.tone === "success"
                ? "border-emerald-200 bg-emerald-50 text-emerald-900"
                : "border-rose-200 bg-rose-50 text-rose-900"
            }`}
          >
            {notification.message}
          </div>
        </div>
      )}

      <header className="shrink-0 border-b border-slate-200 bg-white px-4 py-3">
        <div className="mx-auto flex max-w-[1920px] items-center gap-3">
          <div className="flex h-9 w-9 shrink-0 items-center justify-center rounded-md bg-slate-950 text-white">
            <Bot size={19} />
          </div>
          <div className="min-w-0">
            <div className="truncate text-sm font-bold text-slate-950">GPO Robot Planner</div>
            <div className="truncate text-xs text-slate-500">Планирование и управление маршрутом</div>
          </div>
        </div>
      </header>

      <div className="relative flex min-h-0 flex-1 overflow-hidden">
        <aside
          className={`absolute inset-y-0 left-0 z-40 flex w-[300px] max-w-[calc(100vw-16px)] flex-col border-r border-slate-200 bg-slate-50 shadow-xl transition-transform duration-200 ease-out ${
            sidebarOpen ? "translate-x-0" : "-translate-x-full"
          }`}
        >
          {workspaceSection ? (
            <>
              <div className="shrink-0 border-b border-slate-200 bg-white px-3 py-3">
                <div className="flex items-center justify-between gap-2">
                  <button
                    type="button"
                    onClick={returnToWorkspaceMenu}
                    className="inline-flex h-8 items-center gap-2 rounded-md border border-slate-200 bg-white px-2.5 text-sm font-semibold text-slate-700 shadow-sm transition hover:bg-slate-50"
                  >
                    <ArrowLeft size={16} />
                    Назад
                  </button>
                  <button
                    type="button"
                    onClick={() => setSidebarOpen(false)}
                    title="Скрыть меню"
                    aria-label="Скрыть меню"
                    className="inline-flex h-8 w-8 items-center justify-center rounded-md border border-slate-200 bg-white text-slate-600 shadow-sm transition hover:bg-slate-50"
                  >
                    <PanelLeftClose size={17} />
                  </button>
                </div>
                <div className="mt-3 text-xs font-bold uppercase text-slate-500">Раздел</div>
                <h2 className="mt-1 text-base font-bold text-slate-950">
                  {workspaceSections.find((section) => section.key === workspaceSection)?.label}
                </h2>
              </div>

              <div className="min-h-0 flex-1">
                {showRouteSettings ? (
                  <PlannerLeftSidebar
                    activeTab={workspaceSection}
                    routeTaskKey={routeTaskKey}
                    onRouteTaskChange={handleRouteTaskChange}
                    algorithmKey={algorithmKey}
                    onAlgorithmChange={handleAlgorithmChange}
                    algorithmFields={algorithmFields}
                    selectedAlgorithmParams={selectedAlgorithmParams}
                    onAlgorithmParamChange={updateAlgorithmParam}
                    isOptimizing={isOptimizing}
                    onOptimizeRoute={optimizeRoute}
                    onSendRoute={sendRoute}
                    onAddRandomObstacle={addRandomObstacle}
                    onClearObstacles={clearObstacles}
                    onImportGraph={handleImportGraph}
                    visitEntries={plannerModel.visitEntries}
                    chargeEntries={plannerModel.chargeEntries}
                    plannedVisitEntries={plannerModel.plannedVisitEntries}
                    expandedPoint={expandedPoint}
                    hoveredPointIndex={hoveredPointIndex}
                    onToggleExpandedPoint={setExpandedPoint}
                    onHoverPoint={setHoveredPointIndex}
                    onDeletePoint={deletePoint}
                    onUpdatePointTask={updatePointTask}
                    onClearVisitPoints={() => clearPoints("visit")}
                    onClearChargePoints={() => clearPoints("charge")}
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
                ) : (
                  <PlannerRightSidebar
                    activeTab={workspaceSection}
                    onClearLimitPoints={() => clearPoints("limit")}
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
                    visibleLayers={visibleLayers}
                    onToggleLayer={toggleMapLayer}
                    routeLength={plannerModel.routeLength}
                    mappingSurveyMode={mappingSurveyMode}
                    mappingSurveyModes={MAPPING_SURVEY_MODES}
                    onMappingSurveyModeChange={setMappingSurveyMode}
                    onStartMappingSurvey={startMappingSurvey}
                    onExportMapImage={exportMapImage}
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
            </>
          ) : (
            <>
              <div className="shrink-0 border-b border-slate-200 bg-white px-3 py-3">
                <div className="flex items-center justify-between gap-2">
                  <div>
                    <div className="text-xs font-bold uppercase text-slate-500">Навигация</div>
                    <h2 className="mt-1 text-base font-bold text-slate-950">Разделы</h2>
                  </div>
                  <button
                    type="button"
                    onClick={() => setSidebarOpen(false)}
                    title="Скрыть меню"
                    aria-label="Скрыть меню"
                    className="inline-flex h-8 w-8 shrink-0 items-center justify-center rounded-md border border-slate-200 bg-white text-slate-600 shadow-sm transition hover:bg-slate-50"
                  >
                    <PanelLeftClose size={17} />
                  </button>
                </div>
              </div>
              <nav className="min-h-0 flex-1 overflow-auto p-3" aria-label="Разделы планировщика">
                <div className="space-y-2">
                  {workspaceSections.map((section) => {
                    const Icon = section.icon;
                    return (
                      <button
                        key={section.key}
                        type="button"
                        onClick={() => selectWorkspaceSection(section.key)}
                        className="flex w-full items-center gap-3 rounded-lg border border-slate-200 bg-white px-3 py-3 text-left text-sm font-semibold text-slate-800 shadow-sm transition hover:border-slate-300 hover:bg-slate-50"
                      >
                        <span className="flex h-9 w-9 shrink-0 items-center justify-center rounded-md bg-slate-100 text-slate-700">
                          <Icon size={18} />
                        </span>
                        <span className="min-w-0 flex-1">{section.label}</span>
                      </button>
                    );
                  })}
                </div>
              </nav>
            </>
          )}
        </aside>

        {!sidebarOpen && (
          <button
            type="button"
            onClick={() => setSidebarOpen(true)}
            title="Показать меню"
            aria-label="Показать меню"
            className="absolute left-3 top-3 z-30 inline-flex h-9 w-9 items-center justify-center rounded-md border border-slate-200 bg-white text-slate-700 shadow-md transition hover:bg-slate-50"
          >
            <PanelLeftOpen size={18} />
          </button>
        )}

        <PlannerCanvas
          canvasRef={canvasRef}
          plannerModel={plannerModel}
          optimizedRoute={optimizedRoute}
          hoveredPointIndex={hoveredPointIndex}
          telemetry={telemetry}
          manualObstacles={manualObstacles}
          visibleLayers={visibleLayers}
          onCanvasClick={addPointFromCanvas}
          onCanvasMouseDown={handleCanvasMouseDown}
          onCanvasMouseMove={handleCanvasMouseMove}
          onCanvasMouseUp={finishDragging}
          onCanvasMouseLeave={finishDragging}
        />
      </div>
    </div>
  );
}
