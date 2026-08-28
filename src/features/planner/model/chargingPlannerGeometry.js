import { dist } from "../../../lib/zonePlanner";

export const EPS = 1e-6;
const POINT_KEY_DIGITS = 4;

export const toPoint = (point) => ({ x: Number(point?.x), y: Number(point?.y) });

export const isFinitePoint = (point) =>
  Number.isFinite(point?.x) && Number.isFinite(point?.y);

export const copyPoint = (point) => ({ x: point.x, y: point.y });

export const samePoint = (left, right) =>
  Math.abs(left.x - right.x) <= EPS && Math.abs(left.y - right.y) <= EPS;

export const pointKey = (point) =>
  `${Number(point.x).toFixed(POINT_KEY_DIGITS)},${Number(point.y).toFixed(POINT_KEY_DIGITS)}`;

export const directedEdgeKey = (from, to) => `${pointKey(from)}->${pointKey(to)}`;

export const polylineLength = (route) =>
  route.reduce((sum, point, index) => (index ? sum + dist(route[index - 1], point) : 0), 0);

export const distancePointToSegment = (point, start, end) => {
  const abx = end.x - start.x;
  const aby = end.y - start.y;
  const denominator = abx * abx + aby * aby;
  if (denominator <= EPS) return dist(point, start);

  const t = Math.max(0, Math.min(1, ((point.x - start.x) * abx + (point.y - start.y) * aby) / denominator));
  return dist(point, { x: start.x + abx * t, y: start.y + aby * t });
};

export const dedupeConsecutiveRoute = (route) => {
  if (!route.length) return [];
  const cleaned = [copyPoint(route[0])];
  for (let index = 1; index < route.length; index += 1) {
    if (!samePoint(cleaned[cleaned.length - 1], route[index])) cleaned.push(copyPoint(route[index]));
  }
  return cleaned;
};

export const mergeRoute = (left, right) => {
  if (!left.length) return right.map(copyPoint);
  if (!right.length) return left.map(copyPoint);
  return samePoint(left[left.length - 1], right[0])
    ? left.concat(right.slice(1).map(copyPoint))
    : left.concat(right.map(copyPoint));
};

export const normalizeMandatoryRoute = (route) =>
  dedupeConsecutiveRoute((route || []).map(toPoint).filter(isFinitePoint));

export const normalizeStations = (stations) => {
  const unique = new Map();
  for (const raw of stations || []) {
    const point = toPoint(raw);
    if (isFinitePoint(point)) unique.set(pointKey(point), point);
  }
  return Array.from(unique.values());
};

export const clampBatteryRange = (batteryRange) => {
  const value = Number(batteryRange);
  if (!Number.isFinite(value) || value <= 0) return null;
  return Math.max(0.1, value);
};
