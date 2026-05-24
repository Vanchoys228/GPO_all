import coordinateContract from "../../shared/coordinate-contract.json";
import { ROUTE_WS_URL, TELEMETRY_WS_URL } from "./runtimeConfig";

export { ROUTE_WS_URL, TELEMETRY_WS_URL };
export const INITIAL_TELEMETRY = {
  simulationTime: null,
  x: 0,
  y: 0,
  z: 0,
  yaw: 0,
  navigation: {
    status: "",
    finished: false,
    currentWaypointIndex: 0,
  },
  obstacleTrace: [],
  obstacleMap: {
    cellSize: 0.06,
    cellCount: 0,
    obstacleCellCount: 0,
    freeCellCount: 0,
    mapFile: "obstacle_map.json",
    jsonFile: "obstacle_map.json",
    excelCsvFile: "obstacle_map.csv",
    imageFile: "obstacle_map.png",
    cells: [],
    freeCells: [],
  },
  cameraMap: {
    cellSize: 0.1,
    cellCount: 0,
    obstacleCellCount: 0,
    freeCellCount: 0,
    mapFile: "camera_map.json",
    jsonFile: "camera_map.json",
    excelCsvFile: "camera_map.csv",
    imageFile: "camera_map.png",
    cells: [],
    freeCells: [],
  },
  perception: {
    lidar: null,
    camera: null,
  },
};

const TELEMETRY_MESSAGE_TYPE = coordinateContract.telemetry.messageType;
const TELEMETRY_POSE_KEY = coordinateContract.telemetry.poseKey;

const toFiniteNumber = (value) => {
  if (value === null || value === undefined) return null;
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
};

const pickNumber = (...values) => {
  for (const value of values) {
    const n = toFiniteNumber(value);
    if (n !== null) return n;
  }
  return null;
};

const normalizeObstacleMap = (rawMap, prevMap = INITIAL_TELEMETRY.obstacleMap) => {
  if (!rawMap || typeof rawMap !== "object") return prevMap;

  const normalizeCells = (rawCells, limit) =>
    rawCells
      .map((cell) => ({
        x: pickNumber(cell?.x),
        y: pickNumber(cell?.y),
        confidence: Math.max(0, pickNumber(cell?.confidence, 0) ?? 0),
      }))
      .filter((cell) => cell.x !== null && cell.y !== null)
      .slice(-limit);
  const rawCells = Array.isArray(rawMap.cells) ? rawMap.cells : prevMap?.cells || [];
  const rawFreeCells = Array.isArray(rawMap.freeCells) ? rawMap.freeCells : prevMap?.freeCells || [];
  const cells = normalizeCells(rawCells, 4096);
  const freeCells = normalizeCells(rawFreeCells, 4096);
  const cellSize = pickNumber(rawMap.cellSize, prevMap?.cellSize, INITIAL_TELEMETRY.obstacleMap.cellSize);
  const cellCount = pickNumber(rawMap.cellCount, rawMap.totalCells, cells.length, prevMap?.cellCount, 0);
  const obstacleCellCount = pickNumber(rawMap.obstacleCellCount, cells.length, prevMap?.obstacleCellCount, 0);
  const freeCellCount = pickNumber(rawMap.freeCellCount, freeCells.length, prevMap?.freeCellCount, 0);

  return {
    cellSize: cellSize ?? INITIAL_TELEMETRY.obstacleMap.cellSize,
    cellCount: cellCount ?? cells.length,
    obstacleCellCount: obstacleCellCount ?? cells.length,
    freeCellCount: freeCellCount ?? freeCells.length,
    mapFile:
      typeof rawMap.mapFile === "string" && rawMap.mapFile.trim()
        ? rawMap.mapFile
        : prevMap?.mapFile ?? INITIAL_TELEMETRY.obstacleMap.mapFile,
    jsonFile:
      typeof rawMap.jsonFile === "string" && rawMap.jsonFile.trim()
        ? rawMap.jsonFile
        : prevMap?.jsonFile ?? INITIAL_TELEMETRY.obstacleMap.jsonFile,
    excelCsvFile:
      typeof rawMap.excelCsvFile === "string" && rawMap.excelCsvFile.trim()
        ? rawMap.excelCsvFile
        : prevMap?.excelCsvFile ?? INITIAL_TELEMETRY.obstacleMap.excelCsvFile,
    imageFile:
      typeof rawMap.imageFile === "string" && rawMap.imageFile.trim()
        ? rawMap.imageFile
        : prevMap?.imageFile ?? INITIAL_TELEMETRY.obstacleMap.imageFile,
    cells,
    freeCells,
  };
};

const normalizeCamera = (rawCamera, prevCamera = null) => {
  if (!rawCamera || typeof rawCamera !== "object") return prevCamera ?? null;

  const frameDataUrl =
    typeof rawCamera.frameDataUrl === "string" && rawCamera.frameDataUrl.startsWith("data:image/")
      ? rawCamera.frameDataUrl
      : prevCamera?.frameDataUrl ?? null;

  return {
    enabled: Boolean(rawCamera.enabled),
    width: pickNumber(rawCamera.width, prevCamera?.width, 0) ?? 0,
    height: pickNumber(rawCamera.height, prevCamera?.height, 0) ?? 0,
    fov: pickNumber(rawCamera.fov, prevCamera?.fov, 0) ?? 0,
    frameFile:
      typeof rawCamera.frameFile === "string" && rawCamera.frameFile.trim()
        ? rawCamera.frameFile
        : prevCamera?.frameFile ?? "camera_frame.bmp",
    mimeType:
      typeof rawCamera.mimeType === "string" && rawCamera.mimeType.startsWith("image/")
        ? rawCamera.mimeType
        : prevCamera?.mimeType ?? "image/bmp",
    mode:
      typeof rawCamera.mode === "string" && rawCamera.mode.trim()
        ? rawCamera.mode
        : prevCamera?.mode ?? "webots_camera",
    frameSequence: pickNumber(rawCamera.frameSequence, prevCamera?.frameSequence, 0) ?? 0,
    capturedAt: pickNumber(rawCamera.capturedAt, prevCamera?.capturedAt, 0) ?? 0,
    frameMtimeMs: pickNumber(rawCamera.frameMtimeMs, prevCamera?.frameMtimeMs, null),
    obstacleVisible: Boolean(rawCamera.obstacleVisible),
    obstacleScore: pickNumber(rawCamera.obstacleScore, prevCamera?.obstacleScore, 0) ?? 0,
    obstacleOffset: pickNumber(rawCamera.obstacleOffset, prevCamera?.obstacleOffset, 0) ?? 0,
    obstacleAngle: pickNumber(rawCamera.obstacleAngle, prevCamera?.obstacleAngle, 0) ?? 0,
    obstacleRange: pickNumber(rawCamera.obstacleRange, prevCamera?.obstacleRange, 0) ?? 0,
    detectionCount: pickNumber(rawCamera.detectionCount, prevCamera?.detectionCount, 0) ?? 0,
    frameDataUrl,
  };
};

export const normalizeTelemetry = (raw, prev = INITIAL_TELEMETRY) => {
  if (!raw || typeof raw !== "object") return null;
  if (raw.type && raw.type !== TELEMETRY_MESSAGE_TYPE) return null;

  const pose = raw[TELEMETRY_POSE_KEY] || null;
  const x = pickNumber(pose?.x, raw.x);
  const y = pickNumber(pose?.y, raw.y);
  const z = pickNumber(pose?.z, raw.z, prev.z, 0);
  const yaw = pickNumber(pose?.yaw, raw.yaw, prev.yaw, 0);
  const simulationTime = pickNumber(raw.simulationTime, raw.time, prev.simulationTime);
  const navigation =
    raw.navigation && typeof raw.navigation === "object"
      ? (() => {
          const distanceToTarget = pickNumber(
            raw.navigation.distanceToTarget,
            prev.navigation?.distanceToTarget
          );
          const avoidanceTimeSec = pickNumber(
            raw.navigation.avoidanceTimeSec,
            prev.navigation?.avoidanceTimeSec
          );
          const avoidanceSteps = pickNumber(
            raw.navigation.avoidanceSteps,
            prev.navigation?.avoidanceSteps
          );
          return {
            status:
              typeof raw.navigation.status === "string"
                ? raw.navigation.status
                : prev.navigation?.status ?? "",
            finished: Boolean(raw.navigation.finished),
            currentWaypointIndex:
              pickNumber(raw.navigation.currentWaypointIndex, prev.navigation?.currentWaypointIndex, 0) ?? 0,
            ...(distanceToTarget !== null ? { distanceToTarget } : {}),
            ...(avoidanceTimeSec !== null ? { avoidanceTimeSec } : {}),
            ...(avoidanceSteps !== null ? { avoidanceSteps } : {}),
            ...(typeof raw.navigation.avoidanceActive === "boolean"
              ? { avoidanceActive: raw.navigation.avoidanceActive }
              : {}),
            ...(typeof raw.navigation.offRouteActive === "boolean"
              ? { offRouteActive: raw.navigation.offRouteActive }
              : {}),
          };
        })()
      : prev.navigation ?? INITIAL_TELEMETRY.navigation;
  const rawObstacleTrace =
    (raw.perception && Array.isArray(raw.perception.obstacleTrace)
      ? raw.perception.obstacleTrace
      : null) ??
    (Array.isArray(raw.obstacleTrace) ? raw.obstacleTrace : null);
  const obstacleTrace = Array.isArray(rawObstacleTrace)
    ? rawObstacleTrace
        .map((point) => ({
          x: pickNumber(point?.x),
          y: pickNumber(point?.y),
          confidence: Math.max(0, Math.min(1, pickNumber(point?.confidence, 1) ?? 1)),
        }))
        .filter((point) => point.x !== null && point.y !== null)
        .slice(-520)
    : prev.obstacleTrace || [];
  const lidar = raw?.perception?.lidar ?? prev.perception?.lidar ?? null;
  const camera = normalizeCamera(raw?.perception?.camera ?? raw?.camera, prev.perception?.camera);
  const obstacleMap = normalizeObstacleMap(raw?.obstacleMap, prev.obstacleMap);
  const cameraMap = normalizeObstacleMap(
    raw?.cameraMap,
    prev.cameraMap || INITIAL_TELEMETRY.cameraMap
  );

  if (x === null || y === null) return null;
  return {
    simulationTime,
    x,
    y,
    z,
    yaw,
    navigation,
    obstacleTrace,
    obstacleMap,
    cameraMap,
    perception: {
      lidar,
      camera,
    },
  };
};

export const normalizeAngle = (value) => {
  let angle = value;
  while (angle > Math.PI) angle -= Math.PI * 2;
  while (angle < -Math.PI) angle += Math.PI * 2;
  return angle;
};

export const decodeWsData = async (data) => {
  if (typeof data === "string") return data;
  if (data instanceof Blob) return data.text();
  if (data instanceof ArrayBuffer) return new TextDecoder().decode(data);
  return String(data);
};
