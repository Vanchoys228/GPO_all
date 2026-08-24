import {
  HALF_HEIGHT,
  HALF_WIDTH,
  isInsideMap,
  pointInAnyPolygon,
} from "../../../lib/zonePlanner";

const ROBOT_POINT_CLEARANCE_M = 0.85;
const ROUTE_POINT_CLEARANCE_M = 0.72;
const SEGMENT_ENDPOINT_MARGIN = 0.18;

export const randomBetween = (min, max, random = Math.random) =>
  min + random() * (max - min);

export const pickRandomObstacleCenter = ({
  telemetry,
  optimizedRoute,
  points,
  polygons,
  obstacle,
  random = Math.random,
}) => {
  const allPoints = Array.isArray(points) ? points : [];
  const route = Array.isArray(optimizedRoute) ? optimizedRoute : [];
  const obstacleSizeX = Number(obstacle?.sizeX) || 0.8;
  const obstacleSizeY = Number(obstacle?.sizeY) || 0.8;
  const obstacleRadius = Math.hypot(obstacleSizeX, obstacleSizeY) * 0.5;
  const protectedPointRadius = obstacleRadius + ROBOT_POINT_CLEARANCE_M;
  const protectedRoutePointRadius = obstacleRadius + ROUTE_POINT_CLEARANCE_M;
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
      if (
        Math.hypot(candidate.x - point.x, candidate.y - point.y) <
        protectedRoutePointRadius
      ) {
        return false;
      }
    }

    return true;
  };

  for (let attempt = 0; attempt < totalAttempts; attempt += 1) {
    let candidate = null;
    const useRouteBias = route.length > 1 && attempt < routeBiasAttempts;

    if (useRouteBias) {
      const segmentIndex = Math.floor(random() * (route.length - 1));
      const a = route[segmentIndex];
      const b = route[segmentIndex + 1];
      const t = randomBetween(SEGMENT_ENDPOINT_MARGIN, 1 - SEGMENT_ENDPOINT_MARGIN, random);
      const ax = a.x + (b.x - a.x) * t;
      const ay = a.y + (b.y - a.y) * t;
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const segmentLength = Math.hypot(dx, dy);

      if (segmentLength > 1e-6) {
        const normalX = -dy / segmentLength;
        const normalY = dx / segmentLength;
        const sign = random() < 0.5 ? -1 : 1;
        const lateralOffset = randomBetween(0.22, 0.85, random);
        candidate = {
          x: ax + normalX * lateralOffset * sign,
          y: ay + normalY * lateralOffset * sign,
        };
      }
    }

    if (!candidate) {
      candidate = {
        x: randomBetween(-HALF_WIDTH + 1.2, HALF_WIDTH - 1.2, random),
        y: randomBetween(-HALF_HEIGHT + 1.2, HALF_HEIGHT - 1.2, random),
      };
    }

    if (isSafe(candidate)) return candidate;
  }

  return null;
};
