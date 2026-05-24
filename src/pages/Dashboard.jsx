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
import {
  DEFAULT_ENERGY_OPTIONS,
  SURFACE_PROFILE_OPTIONS,
  analyzeRouteInfluence,
} from "../lib/energyModel";
import PlannerCanvas from "../components/dashboard/PlannerCanvas";
import PlannerLeftSidebar from "../components/dashboard/PlannerLeftSidebar";
import PlannerRightSidebar from "../components/dashboard/PlannerRightSidebar";
import SidebarCollapseRail from "../components/dashboard/SidebarCollapseRail";
import { loadPlannerUiState, savePlannerUiState } from "../lib/plannerUiState";
import * as XLSX from "xlsx";

const ENERGY_SHORTAGE_FALLBACK =
  "Запаса хода не хватает: добавьте станции зарядки или увеличьте запас.";
const CRUISE_SPEED_STORAGE_KEY = "gpo_dashboard_cruise_speed_mps";

const readStoredNumber = (key, fallback, min, max) => {
  if (typeof window === "undefined") return fallback;
  try {
    const stored = window.localStorage.getItem(key);
    if (stored == null) return fallback;
    const parsed = Number(stored);
    if (!Number.isFinite(parsed)) return fallback;
    return Math.max(min, Math.min(max, parsed));
  } catch {
    return fallback;
  }
};

const writeStoredNumber = (key, value) => {
  if (typeof window === "undefined" || !Number.isFinite(value)) return;
  try {
    window.localStorage.setItem(key, String(value));
  } catch {
    // Storage can be disabled in private browser modes.
  }
};

const MAPPING_SURVEY_MODES = [
  { key: "snake", label: "Змейка" },
  { key: "double", label: "Двойной объезд" },
];

const getMappingSurveyModeLabel = (modeKey) =>
  MAPPING_SURVEY_MODES.find((mode) => mode.key === modeKey)?.label ||
  MAPPING_SURVEY_MODES[0].label;

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

const surfaceProfileKeys = new Set(SURFACE_PROFILE_OPTIONS.map((profile) => profile.key));
const DEFAULT_SURFACE_PROFILE_KEY =
  SURFACE_PROFILE_OPTIONS.find((profile) => profile.key !== "neutral")?.key ||
  SURFACE_PROFILE_OPTIONS[0]?.key ||
  "neutral";

const normalizeSurfacePoint = (point) => {
  const x = Number(point?.x);
  const y = Number(point?.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  const normalized = { x, y };
  return isInsideMap(normalized) ? normalized : null;
};

const createSurfaceZoneDraft = (number, surfaceKey = DEFAULT_SURFACE_PROFILE_KEY) => ({
  id: `surface-zone-${number}`,
  name: `Покрытие ${number}`,
  surfaceKey,
  closed: false,
  points: [],
});

const normalizeSurfaceZoneForState = (zone, index) => {
  const points = (Array.isArray(zone?.points) ? zone.points : [])
    .map(normalizeSurfacePoint)
    .filter(Boolean);
  const surfaceKey = surfaceProfileKeys.has(zone?.surfaceKey)
    ? zone.surfaceKey
    : DEFAULT_SURFACE_PROFILE_KEY;

  return {
    id:
      typeof zone?.id === "string" && zone.id.trim()
        ? zone.id.trim()
        : `surface-zone-${index + 1}`,
    name:
      typeof zone?.name === "string" && zone.name.trim()
        ? zone.name.trim()
        : `Покрытие ${index + 1}`,
    surfaceKey,
    closed: zone?.closed === undefined ? points.length >= 3 : Boolean(zone.closed),
    points,
  };
};

const createInitialSurfaceZones = () => {
  const presetZones = DEFAULT_SURFACE_ZONES.map((zone, index) =>
    normalizeSurfaceZoneForState({ ...zone, closed: true }, index)
  );
  const draftNumber = presetZones.length + 1;
  return [...presetZones, createSurfaceZoneDraft(draftNumber)];
};

const deriveNextSurfaceZoneNumber = (zones) => {
  const maxNumber = zones.reduce((best, zone) => {
    const idMatch = String(zone?.id ?? "").match(/surface-zone-(\d+)/i);
    const nameMatch = String(zone?.name ?? "").match(/(\d+)/);
    const candidates = [idMatch?.[1], nameMatch?.[1]]
      .map((value) => Number(value))
      .filter(Number.isFinite);

    return candidates.length ? Math.max(best, ...candidates) : best;
  }, 0);

  return Math.max(1, maxNumber + 1);
};

const normalizeSurfaceZonesForImport = (surfaceZones) => {
  if (!Array.isArray(surfaceZones)) return null;
  const normalized = surfaceZones
    .map((zone, index) => normalizeSurfaceZoneForState(zone, index))
    .filter((zone) => zone.points.length > 0);
  return normalized.length ? normalized : null;
};

const buildSurfaceZonePayload = (surfaceZones) => ({
  type: "surface_zones",
  zones: (Array.isArray(surfaceZones) ? surfaceZones : [])
    .filter((zone) => zone?.closed !== false && Array.isArray(zone?.points) && zone.points.length >= 3)
    .map((zone) => ({
      id: zone.id,
      name: zone.name,
      surfaceKey: surfaceProfileKeys.has(zone.surfaceKey)
        ? zone.surfaceKey
        : DEFAULT_SURFACE_PROFILE_KEY,
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

const OFF_ROUTE_NAVIGATION_STATUSES = new Set([
  "passing_lidar_gap",
  "tracking_lidar_priority",
  "turning_lidar_priority",
  "reacquired_free_space",
]);

const isNavigationOffRoute = (navigation) => {
  if (!navigation || typeof navigation !== "object") return false;
  if (navigation.offRouteActive || navigation.avoidanceActive) return true;
  const status = typeof navigation.status === "string" ? navigation.status : "";
  return status.startsWith("avoiding_") || OFF_ROUTE_NAVIGATION_STATUSES.has(status);
};

const clampCanvasValue = (value, min, max) => Math.max(min, Math.min(max, value));

const loadCanvasImage = (src) =>
  new Promise((resolve) => {
    if (!src) {
      resolve(null);
      return;
    }

    const image = new Image();
    image.onload = () => resolve(image);
    image.onerror = () => resolve(null);
    image.src = src;
  });

const drawCoveredImage = (ctx, image, x, y, width, height) => {
  const scale = Math.max(width / image.width, height / image.height);
  const sourceWidth = width / scale;
  const sourceHeight = height / scale;
  const sourceX = Math.max(0, (image.width - sourceWidth) / 2);
  const sourceY = Math.max(0, (image.height - sourceHeight) / 2);
  ctx.drawImage(image, sourceX, sourceY, sourceWidth, sourceHeight, x, y, width, height);
};

const drawCameraMapExport = async (ctx, exportCanvas, selectedMap, camera) => {
  const cells = Array.isArray(selectedMap?.cells) ? selectedMap.cells : [];
  const freeCells = Array.isArray(selectedMap?.freeCells) ? selectedMap.freeCells : [];
  const width = exportCanvas.width;
  const height = exportCanvas.height;
  const centerX = width / 2;
  const floorTop = height * 0.14;
  const floorBottom = height * 0.94;
  const farWidth = width * 0.40;
  const nearWidth = width * 0.96;
  const cameraFrame = await loadCanvasImage(camera?.frameDataUrl);

  const background = ctx.createLinearGradient(0, 0, width, height);
  background.addColorStop(0, "#07111f");
  background.addColorStop(0.48, "#121827");
  background.addColorStop(1, "#25113a");
  ctx.fillStyle = background;
  ctx.fillRect(0, 0, width, height);

  ctx.save();
  ctx.beginPath();
  ctx.moveTo(centerX - farWidth / 2, floorTop);
  ctx.lineTo(centerX + farWidth / 2, floorTop);
  ctx.lineTo(centerX + nearWidth / 2, floorBottom);
  ctx.lineTo(centerX - nearWidth / 2, floorBottom);
  ctx.closePath();
  ctx.clip();

  const floorGradient = ctx.createLinearGradient(0, floorTop, 0, floorBottom);
  floorGradient.addColorStop(0, "rgba(76,29,149,0.34)");
  floorGradient.addColorStop(0.58, "rgba(30,41,59,0.78)");
  floorGradient.addColorStop(1, "rgba(2,6,23,0.94)");
  ctx.fillStyle = floorGradient;
  ctx.fillRect(0, floorTop, width, floorBottom - floorTop);

  if (cameraFrame) {
    ctx.save();
    ctx.globalAlpha = 0.18;
    ctx.filter = "saturate(1.15) contrast(1.15)";
    drawCoveredImage(ctx, cameraFrame, centerX - nearWidth / 2, floorTop, nearWidth, floorBottom - floorTop);
    ctx.restore();
  }

  for (let depthIndex = 0; depthIndex <= 18; depthIndex += 1) {
    const t = depthIndex / 18;
    const y = floorTop + (floorBottom - floorTop) * t;
    const lineWidth = farWidth + (nearWidth - farWidth) * t;
    const alpha = 0.08 + t * 0.10;
    ctx.strokeStyle = `rgba(148,163,184,${alpha})`;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(centerX - lineWidth / 2, y);
    ctx.lineTo(centerX + lineWidth / 2, y);
    ctx.stroke();
  }
  for (let lateralIndex = -10; lateralIndex <= 10; lateralIndex += 1) {
    const offset = lateralIndex / 20;
    ctx.strokeStyle = "rgba(148,163,184,0.12)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(centerX + offset * farWidth, floorTop);
    ctx.lineTo(centerX + offset * nearWidth, floorBottom);
    ctx.stroke();
  }

  const project = (cell) => {
    const x = clampCanvasValue(Number(cell.x), -HALF_WIDTH, HALF_WIDTH);
    const y = clampCanvasValue(Number(cell.y), -HALF_HEIGHT, HALF_HEIGHT);
    const depth = clampCanvasValue((HALF_HEIGHT - y) / (HALF_HEIGHT * 2), 0, 1);
    const lateral = x / (HALF_WIDTH * 2);
    const widthAtDepth = farWidth + (nearWidth - farWidth) * depth;
    return {
      x: centerX + lateral * widthAtDepth,
      y: floorTop + depth * (floorBottom - floorTop),
      depth,
    };
  };

  const orderedCells = cells
    .map((cell) => ({ cell, point: project(cell) }))
    .sort((left, right) => left.point.depth - right.point.depth);
  const orderedFreeCells = freeCells
    .map((cell) => ({ cell, point: project(cell) }))
    .sort((left, right) => left.point.depth - right.point.depth);

  orderedFreeCells.forEach(({ cell, point }) => {
    const confidence = Math.max(0, Number(cell.confidence) || 0);
    const strength = clampCanvasValue(confidence / 12, 0.12, 1);
    const radius = 3 + point.depth * 9 + strength * 4;
    const alpha = clampCanvasValue(0.10 + strength * 0.28, 0.12, 0.42);

    ctx.fillStyle = `rgba(45,212,191,${alpha})`;
    ctx.strokeStyle = `rgba(103,232,249,${alpha * 0.70})`;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.ellipse(point.x, point.y, radius * 1.75, radius * 0.62, 0, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });

  orderedCells.forEach(({ cell, point }) => {
    const confidence = Math.max(0, Number(cell.confidence) || 0);
    const strength = clampCanvasValue(confidence / 9, 0.24, 1);
    const radius = 4 + point.depth * 8 + strength * 4;
    const columnHeight = 16 + point.depth * 48 + strength * 34;
    const alpha = clampCanvasValue(0.28 + strength * 0.64, 0.32, 0.96);

    const shadow = ctx.createRadialGradient(point.x, point.y, 0, point.x, point.y, radius * 3.2);
    shadow.addColorStop(0, `rgba(217,70,239,${alpha * 0.46})`);
    shadow.addColorStop(0.45, `rgba(124,58,237,${alpha * 0.24})`);
    shadow.addColorStop(1, "rgba(124,58,237,0)");
    ctx.fillStyle = shadow;
    ctx.beginPath();
    ctx.ellipse(point.x, point.y, radius * 2.6, radius * 0.95, 0, 0, Math.PI * 2);
    ctx.fill();

    const column = ctx.createLinearGradient(point.x, point.y - columnHeight, point.x, point.y);
    column.addColorStop(0, `rgba(255,228,245,${alpha})`);
    column.addColorStop(0.42, `rgba(232,121,249,${alpha * 0.96})`);
    column.addColorStop(1, `rgba(126,34,206,${alpha * 0.20})`);
    ctx.strokeStyle = column;
    ctx.lineWidth = Math.max(4, radius * 0.72);
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(point.x, point.y);
    ctx.lineTo(point.x - radius * 0.42, point.y - columnHeight);
    ctx.stroke();

    ctx.fillStyle = `rgba(251,207,232,${alpha})`;
    ctx.beginPath();
    ctx.ellipse(
      point.x - radius * 0.42,
      point.y - columnHeight,
      radius * 0.78,
      radius * 0.46,
      -0.25,
      0,
      Math.PI * 2
    );
    ctx.fill();
  });

  ctx.restore();

  ctx.strokeStyle = "rgba(34,211,238,0.32)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(centerX - farWidth / 2, floorTop);
  ctx.lineTo(centerX + farWidth / 2, floorTop);
  ctx.lineTo(centerX + nearWidth / 2, floorBottom);
  ctx.lineTo(centerX - nearWidth / 2, floorBottom);
  ctx.closePath();
  ctx.stroke();

  const robotX = centerX;
  const robotY = floorBottom - 26;
  ctx.fillStyle = "rgba(52,211,153,0.95)";
  ctx.shadowColor = "rgba(52,211,153,0.85)";
  ctx.shadowBlur = 20;
  ctx.beginPath();
  ctx.arc(robotX, robotY, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.strokeStyle = "rgba(236,253,245,0.92)";
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.strokeStyle = "rgba(45,212,191,0.26)";
  ctx.lineWidth = 2;
  ctx.setLineDash([10, 12]);
  ctx.beginPath();
  ctx.moveTo(robotX, robotY);
  ctx.lineTo(centerX - farWidth / 2, floorTop);
  ctx.moveTo(robotX, robotY);
  ctx.lineTo(centerX + farWidth / 2, floorTop);
  ctx.stroke();
  ctx.setLineDash([]);

  const vignette = ctx.createRadialGradient(centerX, height * 0.5, height * 0.12, centerX, height * 0.5, height * 0.86);
  vignette.addColorStop(0, "rgba(0,0,0,0)");
  vignette.addColorStop(1, "rgba(0,0,0,0.46)");
  ctx.fillStyle = vignette;
  ctx.fillRect(0, 0, width, height);
};

const randomBetween = (min, max) => min + Math.random() * (max - min);
const RANDOM_OBSTACLE_ROBOT_POINT_CLEARANCE_M = 0.85;
const RANDOM_OBSTACLE_ROUTE_POINT_CLEARANCE_M = 0.72;
const RANDOM_OBSTACLE_SEGMENT_ENDPOINT_MARGIN = 0.18;
const createIdleRouteTiming = () => ({
  status: "idle",
  startedAtMs: null,
  actualTimeSec: null,
  seenUnfinished: false,
});
const createEmptyRouteEnergyStats = () => ({
  routeEnergy: 0,
  distanceMeters: 0,
  estimatedTimeSec: 0,
  limitingMaxSpeedMps: DEFAULT_ENERGY_OPTIONS.speedMps,
  averageSlipRisk: 0,
  stationStopCount: 0,
});

const buildRouteEnergyStats = (routeResult) => ({
  routeEnergy: routeResult.routeEnergy,
  distanceMeters: routeResult.routeDistance || 0,
  estimatedTimeSec: routeResult.estimatedTimeSec,
  limitingMaxSpeedMps: routeResult.limitingMaxSpeedMps,
  averageSlipRisk: routeResult.averageSlipRisk,
  stationStopCount: routeResult.stationStopCount || 0,
});

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
      surfaceZones: normalizeSurfaceZonesForImport(rawGraph.surfaceZones),
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
    surfaceZones: normalizeSurfaceZonesForImport(rawGraph.surfaceZones),
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
  const protectedPointRadius = obstacleRadius + RANDOM_OBSTACLE_ROBOT_POINT_CLEARANCE_M;
  const protectedRoutePointRadius = obstacleRadius + RANDOM_OBSTACLE_ROUTE_POINT_CLEARANCE_M;
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
      if (Math.hypot(candidate.x - point.x, candidate.y - point.y) < protectedRoutePointRadius) {
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
      const t = randomBetween(
        RANDOM_OBSTACLE_SEGMENT_ENDPOINT_MARGIN,
        1 - RANDOM_OBSTACLE_SEGMENT_ENDPOINT_MARGIN
      );
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
  const motionProfileTouchedRef = useRef(false);
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
  const [batteryRangeMeters, setBatteryRangeMeters] = useState(
    DEFAULT_BATTERY_RANGE_METERS
  );
  const [cruiseSpeedMps, setCruiseSpeedMps] = useState(() =>
    readStoredNumber(CRUISE_SPEED_STORAGE_KEY, DEFAULT_ENERGY_OPTIONS.speedMps, 0.05, 0.8)
  );
  const [payloadKg, setPayloadKg] = useState(DEFAULT_ENERGY_OPTIONS.payloadKg);
  const [batteryRangeInput, setBatteryRangeInput] = useState(
    String(DEFAULT_BATTERY_RANGE_METERS)
  );
  const [cruiseSpeedInput, setCruiseSpeedInput] = useState(() =>
    formatNumber(
      readStoredNumber(CRUISE_SPEED_STORAGE_KEY, DEFAULT_ENERGY_OPTIONS.speedMps, 0.05, 0.8),
      3
    )
  );
  const [payloadInput, setPayloadInput] = useState(
    formatNumber(DEFAULT_ENERGY_OPTIONS.payloadKg, 2)
  );
  const [routeEnergyStats, setRouteEnergyStats] = useState(createEmptyRouteEnergyStats);
  const [routeTiming, setRouteTiming] = useState(createIdleRouteTiming);
  const [routeOffRouteTiming, setRouteOffRouteTiming] = useState({
    accumulatedSec: 0,
    startedAtMs: null,
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
  const surfaceSyncPayloadText = JSON.stringify(
    buildSurfaceZonePayload(
      plannerModel.surfaceZones.map((zone) => ({
        ...zone,
        points: zone.points.map((point) => ({
          x: Number(point.x.toFixed(4)),
          y: Number(point.y.toFixed(4)),
        })),
      }))
    )
  );
  const energyOptions = useMemo(
    () => ({
      speedMps: cruiseSpeedMps,
      payloadKg,
    }),
    [cruiseSpeedMps, payloadKg]
  );
  const autoRouteSyncToken = `${zoneSyncPayloadText}|${chargePointsRoutingText}|${surfaceSyncPayloadText}|${batteryRangeMeters}`;
  const routeTimingDisplay = {
    status: routeTiming.status,
    actualTimeSec:
      routeTiming.status === "running" && routeTiming.startedAtMs !== null
        ? (Date.now() - routeTiming.startedAtMs) / 1000
        : routeTiming.actualTimeSec,
  };
  const routeOffRouteActive = isNavigationOffRoute(telemetry.navigation);
  const telemetryAvoidanceTimeSec = Number(telemetry.navigation?.avoidanceTimeSec) || 0;
  const localAvoidanceTimeSec =
    routeOffRouteTiming.accumulatedSec +
    (routeOffRouteTiming.startedAtMs !== null
      ? (Date.now() - routeOffRouteTiming.startedAtMs) / 1000
      : 0);
  const routeAvoidanceTimeSec = Math.max(telemetryAvoidanceTimeSec, localAvoidanceTimeSec);
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

  const startRouteTiming = () => {
    setRouteTiming({
      status: "running",
      startedAtMs: Date.now(),
      actualTimeSec: null,
      seenUnfinished: false,
    });
    setRouteOffRouteTiming({
      accumulatedSec: 0,
      startedAtMs: null,
    });
  };

  const resetRouteTiming = () => {
    setRouteTiming(createIdleRouteTiming());
    setRouteOffRouteTiming({
      accumulatedSec: 0,
      startedAtMs: null,
    });
  };

  const handleImportGraph = (rawGraph, sourceName = "graph.json") => {
    const imported = normalizeImportedGraph(rawGraph);
    const importedZones =
      imported.limitZones.length > 0 ? imported.limitZones : [INITIAL_ZONE];

    setPoints(imported.points);
    setLimitZones(importedZones);
    if (imported.surfaceZones) {
      const nextSurfaceZones = imported.surfaceZones;
      setSurfaceZones(nextSurfaceZones);
      setActiveSurfaceZoneId(nextSurfaceZones[0]?.id || "");
      setActiveSurfaceProfileKey(
        nextSurfaceZones[0]?.surfaceKey || DEFAULT_SURFACE_PROFILE_KEY
      );
      setNextSurfaceZoneNumber(deriveNextSurfaceZoneNumber(nextSurfaceZones));
    }
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
    setRouteEnergyStats(createEmptyRouteEnergyStats());
    resetRouteTiming();

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

  const handleImportFile = async (file) => {
    const extension = file.name.split(".").pop()?.toLowerCase() || "";

    if (extension === "json") {
      const rawText = await file.text();
      handleImportGraph(JSON.parse(rawText), file.name);
      return;
    }

    if (!["xlsx", "xls", "csv"].includes(extension)) {
      throw new Error("Поддерживаются JSON, Excel и CSV.");
    }

    const workbook = XLSX.read(await file.arrayBuffer(), {
      type: "array",
      raw: false,
    });
    const firstSheetName = workbook.SheetNames[0];
    const sheet = firstSheetName ? workbook.Sheets[firstSheetName] : null;
    if (!sheet) throw new Error("В файле не найден лист с точками.");

    const rows = XLSX.utils.sheet_to_json(sheet, { header: 1, defval: "" });
    if (!rows.length) throw new Error("Файл пуст.");

    const parseNumber = (value) => {
      const normalized = String(value ?? "").trim().replace(",", ".");
      if (!normalized) return null;
      const parsed = Number(normalized);
      return Number.isFinite(parsed) ? parsed : null;
    };
    const normalizeHeader = (value) => String(value ?? "").trim().toLowerCase();
    const firstRow = rows[0] || [];
    const firstRowHasCoordinates =
      parseNumber(firstRow[0]) !== null && parseNumber(firstRow[1]) !== null;
    const headers = firstRowHasCoordinates ? [] : firstRow.map(normalizeHeader);
    const dataRows = firstRowHasCoordinates ? rows : rows.slice(1);
    const findColumn = (aliases, fallback) => {
      const index = headers.findIndex((header) => aliases.includes(header));
      return index >= 0 ? index : fallback;
    };

    const xCol = findColumn(["x", "х", "xcoord", "x coordinate", "координата x"], 0);
    const yCol = findColumn(["y", "у", "ycoord", "y coordinate", "координата y"], 1);
    const kindCol = findColumn(["kind", "type", "тип", "роль"], -1);
    const taskCol = findColumn(["task", "operation", "операция", "задача"], -1);

    const points = dataRows
      .map((row) => {
        const x = parseNumber(row?.[xCol]);
        const y = parseNumber(row?.[yCol]);
        if (x === null || y === null) return null;

        const kindText = kindCol >= 0 ? normalizeHeader(row?.[kindCol]) : "";
        const kind =
          kindText.includes("charge") || kindText.includes("зар")
            ? "charge"
            : kindText.includes("limit") || kindText.includes("zone") || kindText.includes("зон")
              ? "limit"
              : "visit";
        return {
          x,
          y,
          kind,
          zoneId: kind === "limit" ? INITIAL_ZONE.id : null,
          task: kind === "visit" && taskCol >= 0 ? String(row?.[taskCol] || DEFAULT_POINT_TASK) : null,
        };
      })
      .filter(Boolean);

    if (!points.length) {
      throw new Error("В файле не найдено валидных координат.");
    }

    handleImportGraph(
      {
        points,
        limitZones: points.some((point) => point.kind === "limit") ? [INITIAL_ZONE] : [],
      },
      file.name
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
    if (routeTiming.status !== "running") return;

    const finished = Boolean(telemetry.navigation?.finished);
    if (!finished && !routeTiming.seenUnfinished) {
      setRouteTiming((prev) =>
        prev.status === "running" ? { ...prev, seenUnfinished: true } : prev
      );
      return;
    }

    if (finished && routeTiming.seenUnfinished && routeTiming.startedAtMs !== null) {
      const actualTimeSec = (Date.now() - routeTiming.startedAtMs) / 1000;
      setRouteTiming({
        status: "finished",
        startedAtMs: routeTiming.startedAtMs,
        actualTimeSec,
        seenUnfinished: true,
      });
    }
  }, [
    routeTiming.seenUnfinished,
    routeTiming.startedAtMs,
    routeTiming.status,
    telemetry.navigation?.finished,
  ]);

  useEffect(() => {
    const now = Date.now();
    setRouteOffRouteTiming((prev) => {
      if (routeTiming.status !== "running") {
        return prev.startedAtMs !== null
          ? {
              accumulatedSec: prev.accumulatedSec + (now - prev.startedAtMs) / 1000,
              startedAtMs: null,
            }
          : prev;
      }

      if (routeOffRouteActive && prev.startedAtMs === null) {
        return {
          ...prev,
          startedAtMs: now,
        };
      }

      if (!routeOffRouteActive && prev.startedAtMs !== null) {
        return {
          accumulatedSec: prev.accumulatedSec + (now - prev.startedAtMs) / 1000,
          startedAtMs: null,
        };
      }

      return prev;
    });
  }, [routeOffRouteActive, routeTiming.status, telemetry.navigation?.status]);

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
    sendRouteChannelPayload(routeWsRef, JSON.parse(surfaceSyncPayloadText));
  }, [surfaceSyncPayloadText]);

  useEffect(() => {
    writeStoredNumber(CRUISE_SPEED_STORAGE_KEY, cruiseSpeedMps);
  }, [cruiseSpeedMps]);

  useEffect(() => {
    if (!motionProfileTouchedRef.current) {
      motionProfileTouchedRef.current = true;
      return;
    }

    sendRouteChannelPayload(routeWsRef, {
      type: "motion_profile",
      motion: {
        cruiseSpeedMps,
        payloadKg,
        batteryRange: batteryRangeMeters,
      },
    });
  }, [batteryRangeMeters, cruiseSpeedMps, payloadKg]);

  useEffect(() => {
    if (!routeSeed.length) {
      setOptimizedRoute([]);
      setEnergyWarning("");
      setRouteEnergyStats(createEmptyRouteEnergyStats());
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
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      setStatus(nextRoute.error || "Маршрут недостижим при текущих ограничениях.");
      return;
    }
    setEnergyWarning("");
    setOptimizedRoute(nextRoute.route);
    setRouteEnergyStats(buildRouteEnergyStats(nextRoute));
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
      setRouteEnergyStats(createEmptyRouteEnergyStats());
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
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      setStatus(rebuilt.error || "Невозможно безопасно перестроить маршрут.");
      return;
    }
    setEnergyWarning("");
    setRouteEnergyStats(buildRouteEnergyStats(rebuilt));
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
        startRouteTiming();
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
      setRouteEnergyStats(createEmptyRouteEnergyStats());
      resetRouteTiming();
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

  const createSurfaceZone = () => {
    const zone = createSurfaceZoneDraft(nextSurfaceZoneNumber, activeSurfaceProfileKey);
    setSurfaceZones((prev) => [...prev, zone]);
    setActiveSurfaceZoneId(zone.id);
    setNextSurfaceZoneNumber((prev) => prev + 1);
    setActivePointKind("surface");
    setStatus(`Создана зона покрытия: ${zone.name}.`);
  };

  const selectSurfaceZone = (zoneId) => {
    const target = surfaceZones.find((zone) => zone.id === zoneId);
    if (!target) return;
    setActiveSurfaceZoneId(zoneId);
    setActiveSurfaceProfileKey(target.surfaceKey);
    setActivePointKind("surface");
  };

  const updateActiveSurfaceProfile = (surfaceKey) => {
    const nextKey = surfaceProfileKeys.has(surfaceKey)
      ? surfaceKey
      : DEFAULT_SURFACE_PROFILE_KEY;
    setActiveSurfaceProfileKey(nextKey);
    if (!activeSurfaceZoneId) return;
    setSurfaceZones((prev) =>
      prev.map((zone) =>
        zone.id === activeSurfaceZoneId ? { ...zone, surfaceKey: nextKey } : zone
      )
    );
    clearRouteState({ dropSolvedRoute: false });
  };

  const toggleSurfaceZoneClosed = (zoneId) => {
    const target = surfaceZones.find((zone) => zone.id === zoneId);
    if (!target) return;

    if (!target.closed && target.points.length < 3) {
      setStatus("Чтобы замкнуть покрытие, нужно минимум три точки.");
      return;
    }

    setSurfaceZones((prev) =>
      prev.map((zone) =>
        zone.id === zoneId ? { ...zone, closed: !zone.closed } : zone
      )
    );
    setActiveSurfaceZoneId(zoneId);
    setActivePointKind("surface");
    clearRouteState({ dropSolvedRoute: false });
    setStatus(
      target.closed
        ? `${target.name} открыта для редактирования.`
        : `${target.name} замкнута и будет учитываться в расчёте.`
    );
  };

  const clearSurfaceZone = (zoneId) => {
    setSurfaceZones((prev) =>
      prev.map((zone) =>
        zone.id === zoneId ? { ...zone, points: [], closed: false } : zone
      )
    );
    setActiveSurfaceZoneId(zoneId);
    setActivePointKind("surface");
    clearRouteState({ dropSolvedRoute: false });
    setStatus("Точки выбранного покрытия очищены.");
  };

  const removeSurfaceZone = (zoneId) => {
    const nextZones = surfaceZones.filter((zone) => zone.id !== zoneId);
    if (!nextZones.length) {
      const fallback = createSurfaceZoneDraft(1, activeSurfaceProfileKey);
      setSurfaceZones([fallback]);
      setActiveSurfaceZoneId(fallback.id);
      setNextSurfaceZoneNumber(2);
    } else {
      setSurfaceZones(nextZones);
      if (activeSurfaceZoneId === zoneId) {
        setActiveSurfaceZoneId(nextZones[0].id);
        setActiveSurfaceProfileKey(nextZones[0].surfaceKey);
      }
      setNextSurfaceZoneNumber(deriveNextSurfaceZoneNumber(nextZones));
    }
    clearRouteState({ dropSolvedRoute: false });
    setStatus("Зона покрытия удалена.");
  };

  const clearAllSurfaceZones = () => {
    const fallback = createSurfaceZoneDraft(1, activeSurfaceProfileKey);
    setSurfaceZones([fallback]);
    setActiveSurfaceZoneId(fallback.id);
    setNextSurfaceZoneNumber(2);
    clearRouteState({ dropSolvedRoute: false });
    setStatus("Все зоны покрытий очищены.");
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

    if (activePointKind === "surface") {
      if (!activeSurfaceZone) {
        setStatus("Сначала создайте зону покрытия.");
        return;
      }
      if (activeSurfaceZone.closed) {
        setStatus("Покрытие уже замкнуто. Откройте его, чтобы добавить точки.");
        return;
      }

      setSurfaceZones((prev) =>
        prev.map((zone) =>
          zone.id === activeSurfaceZone.id
            ? { ...zone, surfaceKey: activeSurfaceProfileKey, points: [...zone.points, point] }
            : zone
        )
      );
      clearRouteState({ dropSolvedRoute: false });
      setStatus(`Добавлена точка в ${activeSurfaceZone.name}.`);
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
    setRouteEnergyStats(createEmptyRouteEnergyStats());
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
        setRouteEnergyStats(createEmptyRouteEnergyStats());
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
      setRouteEnergyStats(buildRouteEnergyStats(routed));
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
      setRouteEnergyStats(createEmptyRouteEnergyStats());
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
      setRouteEnergyStats(buildRouteEnergyStats(rebuiltForController));
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
      startRouteTiming();
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

  const startMappingSurvey = () => {
    const payload = {
      type: "start_mapping_survey",
      commandId: Date.now(),
      clearMap: true,
      mode: mappingSurveyMode,
      field: {
        minX: -HALF_WIDTH,
        maxX: HALF_WIDTH,
        minY: -HALF_HEIGHT,
        maxY: HALF_HEIGHT,
      },
      motion: {
        cruiseSpeedMps: 0.8,
        payloadKg,
        batteryRange: batteryRangeMeters,
      },
    };
    const modeLabel = getMappingSurveyModeLabel(mappingSurveyMode);

    sendRouteChannelPayload(routeWsRef, payload, {
      onSent: () => {
        setStatus(
          `Запущено обследование карты: скорость 0.8 м/с, сначала периметр, затем "${modeLabel}".`
        );
      },
      onError: () => {
        setStatus("Не удалось отправить команду объезда карты.");
      },
    });
  };

  const requestMapExport = () => {
    const hasLidarMap = Boolean(telemetry.obstacleMap?.cells?.length);
    const cameraObstacleCells = Array.isArray(telemetry.cameraMap?.cells)
      ? telemetry.cameraMap.cells.length
      : 0;
    const cameraFreeCells = Array.isArray(telemetry.cameraMap?.freeCells)
      ? telemetry.cameraMap.freeCells.length
      : 0;
    const hasCameraMap = Boolean(cameraObstacleCells + cameraFreeCells);
    if (!hasLidarMap && !hasCameraMap) {
      setStatus("Пока нет накопленной карты препятствий для экспорта.");
      return;
    }
    setMapExportPromptOpen(true);
  };

  const setSidebarCollapsed = (key, collapsed) => {
    setPlannerUiState((prev) => {
      const next = { ...prev, [key]: collapsed };
      savePlannerUiState(next);
      return next;
    });
  };

  const exportMapImage = async (variant = "lidar") => {
    const normalizedVariant = variant === "camera" ? "camera" : "lidar";
    const selectedMap =
      normalizedVariant === "camera" ? telemetry.cameraMap : telemetry.obstacleMap;
    const selectedCells = Array.isArray(selectedMap?.cells) ? selectedMap.cells : [];
    const selectedFreeCells = Array.isArray(selectedMap?.freeCells) ? selectedMap.freeCells : [];
    if (!selectedCells.length && !selectedFreeCells.length) {
      setStatus(
        normalizedVariant === "camera"
          ? "Камерная карта пока пустая."
          : "Лидарная карта пока пустая."
      );
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

    if (normalizedVariant === "camera") {
      await drawCameraMapExport(ctx, exportCanvas, selectedMap, telemetry.perception?.camera);
    } else {
      drawPlannerBackground(ctx, [], { annotate: false });

      const rawCellSize = Number(selectedMap?.cellSize);
      const cellSize =
        Number.isFinite(rawCellSize) && rawCellSize > 0 ? rawCellSize : 0.06;
      const cellCanvasSize = Math.max(3, cellSize * SCALE * 0.92);

      selectedCells.forEach((cell) => {
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
    }

    const link = document.createElement("a");
    const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
    const filePrefix = normalizedVariant === "camera" ? "camera-map" : "lidar-map";
    const fileName = `${filePrefix}-${timestamp}.png`;
    link.href = exportCanvas.toDataURL("image/png");
    link.download = fileName;
    link.click();
    setMapExportPromptOpen(false);
    setStatus(
      normalizedVariant === "camera"
        ? `Камерная карта сохранена в PNG: ${link.download}`
        : `Лидарная карта сохранена в PNG: ${link.download}`
    );
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
