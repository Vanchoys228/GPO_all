import { SURFACE_PROFILES, SURFACE_ZONE_PRESETS, getSurfaceProfileByKey } from "./energyProfiles";

const EPS = 1e-9;

const pointOnSegment = (point, a, b) => {
  const cross = (point.y - a.y) * (b.x - a.x) - (point.x - a.x) * (b.y - a.y);
  if (Math.abs(cross) > EPS) return false;
  return (point.x - a.x) * (point.x - b.x) + (point.y - a.y) * (point.y - b.y) <= EPS;
};

const pointInPolygon = (point, polygon) => {
  if (!Array.isArray(polygon) || polygon.length < 3) return false;
  let inside = false;
  for (let index = 0, previous = polygon.length - 1; index < polygon.length; previous = index, index += 1) {
    const currentPoint = polygon[index];
    const previousPoint = polygon[previous];
    if (pointOnSegment(point, currentPoint, previousPoint)) return true;
    const intersects = currentPoint.y > point.y !== previousPoint.y > point.y &&
      point.x < ((previousPoint.x - currentPoint.x) * (point.y - currentPoint.y)) / (previousPoint.y - currentPoint.y + EPS) + currentPoint.x;
    if (intersects) inside = !inside;
  }
  return inside;
};

export const normalizeSurfaceZones = (zones) =>
  (Array.isArray(zones) ? zones : [])
    .map((zone) => ({
      ...zone,
      points: Array.isArray(zone?.points)
        ? zone.points.map((point) => ({ x: Number(point?.x), y: Number(point?.y) }))
          .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
        : [],
    }))
    .filter((zone) => zone.points.length >= 3 && zone.closed !== false);

export const resolveSurfaceAtPoint = (point, zones = SURFACE_ZONE_PRESETS) => {
  const normalizedZones = normalizeSurfaceZones(zones);
  for (let index = normalizedZones.length - 1; index >= 0; index -= 1) {
    const zone = normalizedZones[index];
    if (pointInPolygon(point, zone.points)) return { zone, profile: getSurfaceProfileByKey(zone.surfaceKey) };
  }
  return { zone: null, profile: SURFACE_PROFILES.neutral };
};
