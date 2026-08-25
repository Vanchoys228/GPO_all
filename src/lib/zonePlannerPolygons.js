import { HALF_HEIGHT, HALF_WIDTH } from "./zonePlannerCoordinates";
import {
  dist,
  pointEquals,
  pointInPolygon,
  segmentsIntersect,
} from "./zonePlannerGeometry";

export const SAFE_POINT_MARGIN = 0.65;
export const ROUTE_CLEARANCE_MARGIN = 0.55;
export const MAP_TRAVERSAL_MARGIN = 0.18;
const EPS = 1e-9;

const copyPoint = (point) => ({ ...point });
const clampNumber = (value, min, max) => Math.max(min, Math.min(max, value));
const clampPointToMap = (point) => ({
  x: clampNumber(point.x, -HALF_WIDTH + SAFE_POINT_MARGIN, HALF_WIDTH - SAFE_POINT_MARGIN),
  y: clampNumber(point.y, -HALF_HEIGHT + SAFE_POINT_MARGIN, HALF_HEIGHT - SAFE_POINT_MARGIN),
});

const getClosestPointOnSegment = (point, a, b) => {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const lengthSquared = abx * abx + aby * aby;
  if (lengthSquared <= EPS) return copyPoint(a);
  const t = clampNumber(
    ((point.x - a.x) * abx + (point.y - a.y) * aby) / lengthSquared,
    0,
    1
  );
  return { x: a.x + abx * t, y: a.y + aby * t };
};

const distancePointToSegment = (point, a, b) =>
  dist(point, getClosestPointOnSegment(point, a, b));

const distancePointToPolygonEdges = (point, polygon) => {
  if (polygon.length < 2) return Number.POSITIVE_INFINITY;
  let best = Number.POSITIVE_INFINITY;
  for (let i = 0; i < polygon.length; i += 1) {
    best = Math.min(
      best,
      distancePointToSegment(point, polygon[i], polygon[(i + 1) % polygon.length])
    );
  }
  return best;
};

const isPointNearPolygon = (point, polygon, margin = 0) => {
  if (pointInPolygon(point, polygon)) return true;
  return margin > EPS && distancePointToPolygonEdges(point, polygon) <= margin + EPS;
};

export const findBlockingPolygon = (point, polygons, margin = 0) =>
  polygons.find((polygon) => isPointNearPolygon(point, polygon.points, margin));

const projectPointOutsidePolygon = (point, polygon, margin) => {
  let bestBoundaryPoint = null;
  let bestCandidate = null;
  let bestDistance = Number.POSITIVE_INFINITY;
  const targetMargin = margin + 0.08;
  for (let i = 0; i < polygon.length; i += 1) {
    const a = polygon[i];
    const b = polygon[(i + 1) % polygon.length];
    const boundaryPoint = getClosestPointOnSegment(point, a, b);
    const distance = dist(point, boundaryPoint);
    if (distance > bestDistance + EPS) continue;
    const edgeX = b.x - a.x;
    const edgeY = b.y - a.y;
    const edgeLength = Math.hypot(edgeX, edgeY);
    if (edgeLength <= EPS) continue;
    const candidates = [
      { x: -edgeY / edgeLength, y: edgeX / edgeLength },
      { x: edgeY / edgeLength, y: -edgeX / edgeLength },
    ]
      .map((normal) =>
        clampPointToMap({
          x: boundaryPoint.x + normal.x * targetMargin,
          y: boundaryPoint.y + normal.y * targetMargin,
        })
      )
      .filter((candidate) => !isPointNearPolygon(candidate, polygon, margin));
    if (!candidates.length) continue;
    bestBoundaryPoint = boundaryPoint;
    bestCandidate = candidates.sort((left, right) => dist(point, left) - dist(point, right))[0];
    bestDistance = distance;
  }
  if (bestCandidate) return bestCandidate;
  if (bestBoundaryPoint) return clampPointToMap(bestBoundaryPoint);
  return clampPointToMap(point);
};

export const projectPointOutsidePolygons = (point, polygons, margin = SAFE_POINT_MARGIN) => {
  let current = copyPoint(point);
  let adjusted = false;
  for (let step = 0; step < polygons.length + 4; step += 1) {
    const polygon = findBlockingPolygon(current, polygons, margin);
    if (!polygon) break;
    current = projectPointOutsidePolygon(current, polygon.points, margin);
    adjusted = true;
  }
  return { point: current, adjusted };
};

const distanceBetweenSegments = (a, b, c, d) => {
  if (segmentsIntersect(a, b, c, d)) return 0;
  return Math.min(
    distancePointToSegment(a, c, d),
    distancePointToSegment(b, c, d),
    distancePointToSegment(c, a, b),
    distancePointToSegment(d, a, b)
  );
};

export const isInsideTraversableMap = (point, margin = MAP_TRAVERSAL_MARGIN) =>
  point.x >= -HALF_WIDTH + margin &&
  point.x <= HALF_WIDTH - margin &&
  point.y >= -HALF_HEIGHT + margin &&
  point.y <= HALF_HEIGHT - margin;

const pointAtSegment = (a, b, t) => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
});

export const segmentClear = (a, b, polygons, margin = ROUTE_CLEARANCE_MARGIN) => {
  if (!isInsideTraversableMap(a) || !isInsideTraversableMap(b)) return false;
  if (pointEquals(a, b)) return !findBlockingPolygon(a, polygons, margin);
  for (const polygon of polygons) {
    const points = polygon.points;
    if (
      isPointNearPolygon(a, points, Math.max(0, margin - 0.03)) ||
      isPointNearPolygon(b, points, Math.max(0, margin - 0.03))
    ) return false;
    for (const t of [0.25, 0.5, 0.75]) {
      if (pointInPolygon(pointAtSegment(a, b, t), points)) return false;
    }
    for (let i = 0; i < points.length; i += 1) {
      if (
        distanceBetweenSegments(a, b, points[i], points[(i + 1) % points.length]) <=
        margin + EPS
      ) return false;
    }
  }
  return true;
};

export const routeCrossesAnyLimitPolygon = (route, polygons) =>
  polygons.some((polygon) => {
    if (route.length < 2 || polygon.points.length < 3) return false;
    for (let i = 1; i < route.length; i += 1) {
      if (!segmentClear(route[i - 1], route[i], [polygon])) return true;
    }
    return false;
  });
