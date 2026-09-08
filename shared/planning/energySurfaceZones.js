import { SURFACE_PROFILES, SURFACE_ZONE_PRESETS, getSurfaceProfileByKey } from "./energyProfiles.js";

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

// Split at polygon edges so a narrow surface cannot disappear between waypoints.
export const splitSegmentBySurfaces = (from, to, zones) => {
  const dx = to.x - from.x;
  const dy = to.y - from.y;
  const lengthSquared = dx * dx + dy * dy;
  if (lengthSquared <= EPS) return [];
  const cuts = [0, 1];
  for (const zone of normalizeSurfaceZones(zones)) {
    for (let i = 0; i < zone.points.length; i += 1) {
      const a = zone.points[i];
      const b = zone.points[(i + 1) % zone.points.length];
      const ex = b.x - a.x;
      const ey = b.y - a.y;
      const ax = a.x - from.x;
      const ay = a.y - from.y;
      const denominator = dx * ey - dy * ex;
      if (Math.abs(denominator) > EPS) {
        const t = (ax * ey - ay * ex) / denominator;
        const u = (ax * dy - ay * dx) / denominator;
        if (t > 0 && t < 1 && u >= -EPS && u <= 1 + EPS) cuts.push(t);
      } else if (Math.abs(ax * dy - ay * dx) <= EPS) {
        for (const p of [a, b]) {
          const t = ((p.x - from.x) * dx + (p.y - from.y) * dy) / lengthSquared;
          if (t > 0 && t < 1) cuts.push(t);
        }
      }
    }
  }
  cuts.sort((a, b) => a - b);
  const parts = [];
  for (let i = 1; i < cuts.length; i += 1) {
    if (cuts[i] - cuts[i - 1] <= EPS) continue;
    const t = (cuts[i] + cuts[i - 1]) / 2;
    parts.push({ fraction: cuts[i] - cuts[i - 1],
      profile: resolveSurfaceAtPoint({ x: from.x + t * dx, y: from.y + t * dy }, zones).profile });
  }
  return parts;
};
