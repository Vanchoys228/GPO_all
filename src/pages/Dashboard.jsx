import { useEffect, useRef, useState } from "react";
import {
  ALGORITHM_OPTIONS,
  TASK_OPTIONS,
  getAlgorithmFields,
  getAlgorithmLabel,
  getDefaultAlgorithmParams,
  getTaskLabel,
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

  const handleImportGraph = (rawGraph, sourceName = "graph.json") => {
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
    setEnergyWarning("");
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

    const visitCount = imported.points.filter((point) => point.kind === "visit").length;
    const chargeCount = imported.points.filter((point) => point.kind === "charge").length;
    const zonePointCount = imported.points.filter((point) => point.kind === "limit").length;
    setStatus(
      `Граф импортирован из ${sourceName}: точек посещения ${visitCount}, зарядок ${chargeCount}, точек зон ${zonePointCount}.`
    );
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
      setStatus("Не удалось подобрать безопасное место для случайного препятствия.");
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

    sendRouteChannelPayload(routeWsRef, payload, {
      onSent: () => {
        setStatus(
          `Случайное препятствие добавлено: (${payload.obstacle.x.toFixed(2)}, ${payload.obstacle.y.toFixed(2)}).`
        );
      },
      onError: () => {
        setStatus("Не удалось отправить команду добавления препятствия.");
      },
    });
  };

  const exportMapImage = () => {
    if (!telemetry.obstacleMap?.cells?.length) {
      setStatus("Пока нет накопленной карты препятствий для экспорта.");
      return;
    }

    const exportCanvas = document.createElement("canvas");
    exportCanvas.width = CANVAS_WIDTH;
    exportCanvas.height = CANVAS_HEIGHT;

    const ctx = exportCanvas.getContext("2d");
    if (!ctx) {
      setStatus("Не удалось подготовить PNG-экспорт карты.");
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
    setStatus(`Карта сохранена в PNG: ${link.download}`);
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
        onAddRandomObstacle={addRandomObstacle}
        onImportGraph={handleImportGraph}
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
    </div>
  );
}
