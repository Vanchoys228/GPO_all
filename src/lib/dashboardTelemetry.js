import coordinateContract from "../../shared/coordinate-contract.json";
import { ROUTE_WS_URL, TELEMETRY_WS_URL } from "./runtimeConfig";

export { ROUTE_WS_URL, TELEMETRY_WS_URL };
export const INITIAL_TELEMETRY = {
  x: 0,
  y: 0,
  z: 0,
  yaw: 0,
  obstacleTrace: [],
  perception: {
    lidar: null,
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

export const normalizeTelemetry = (raw, prev = INITIAL_TELEMETRY) => {
  if (!raw || typeof raw !== "object") return null;
  if (raw.type && raw.type !== TELEMETRY_MESSAGE_TYPE) return null;

  const pose = raw[TELEMETRY_POSE_KEY] || null;
  const x = pickNumber(pose?.x, raw.x);
  const y = pickNumber(pose?.y, raw.y);
  const z = pickNumber(pose?.z, raw.z, prev.z, 0);
  const yaw = pickNumber(pose?.yaw, raw.yaw, prev.yaw, 0);
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
        }))
        .filter((point) => point.x !== null && point.y !== null)
    : prev.obstacleTrace || [];
  const lidar = raw?.perception?.lidar ?? prev.perception?.lidar ?? null;

  if (x === null || y === null) return null;
  return {
    x,
    y,
    z,
    yaw,
    obstacleTrace,
    perception: {
      lidar,
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
