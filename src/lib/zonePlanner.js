export const CANVAS_WIDTH = 1180;
export const CANVAS_HEIGHT = 920;
export const MAP_WORLD_WIDTH = 44;
export const MAP_WORLD_HEIGHT = 34;
export const MAP_PADDING = 44;
export const HALF_WIDTH = MAP_WORLD_WIDTH / 2;
export const HALF_HEIGHT = MAP_WORLD_HEIGHT / 2;
export const SCALE = Math.min(
  (CANVAS_WIDTH - MAP_PADDING * 2) / MAP_WORLD_WIDTH,
  (CANVAS_HEIGHT - MAP_PADDING * 2) / MAP_WORLD_HEIGHT
);
export const DRAWING_WIDTH = MAP_WORLD_WIDTH * SCALE;
export const DRAWING_HEIGHT = MAP_WORLD_HEIGHT * SCALE;
export const DRAWING_LEFT = (CANVAS_WIDTH - DRAWING_WIDTH) / 2;
export const DRAWING_TOP = (CANVAS_HEIGHT - DRAWING_HEIGHT) / 2;

export const POINT_KIND_META = {
  visit: {
    key: "visit",
    label: "Точки посещения",
    shortLabel: "V",
    color: "#dc2626",
    bg: "bg-rose-50",
    border: "border-rose-200",
    text: "text-rose-700",
  },
  charge: {
    key: "charge",
    label: "Станция зарядки",
    shortLabel: "C",
    color: "#f59e0b",
    bg: "bg-amber-50",
    border: "border-amber-200",
    text: "text-amber-700",
  },
  limit: {
    key: "limit",
    label: "Ограничивающий контур",
    shortLabel: "Z",
    color: "#2563eb",
    bg: "bg-blue-50",
    border: "border-blue-200",
    text: "text-blue-700",
  },
};

export const POINT_KIND_OPTIONS = [
  POINT_KIND_META.visit,
  POINT_KIND_META.charge,
  POINT_KIND_META.limit,
];

export const POINT_TASKS = [
  "Ожидание 2 сек",
  "Сканирование",
  "Забрать объект",
  "Сбросить объект",
  "Сделать фото",
];

export const DEFAULT_POINT_TASK = POINT_TASKS[0];
export const CONTROLLER_MIN_SEGMENT = 0.12;
export const SAFE_POINT_MARGIN = 0.65;
export const ROUTE_CLEARANCE_MARGIN = 0.55;
const MAP_TRAVERSAL_MARGIN = 0.18;
const GRID_PATH_STEP = 0.3;
const COLLINEAR_EPS = 1e-4;

const EPS = 1e-9;
const ZONE_COLORS = [
  { stroke: "#2563eb", fill: "rgba(37, 99, 235, 0.12)", badge: "bg-blue-500" },
  { stroke: "#7c3aed", fill: "rgba(124, 58, 237, 0.12)", badge: "bg-violet-500" },
  { stroke: "#0891b2", fill: "rgba(8, 145, 178, 0.12)", badge: "bg-cyan-500" },
  { stroke: "#ea580c", fill: "rgba(234, 88, 12, 0.12)", badge: "bg-orange-500" },
  { stroke: "#16a34a", fill: "rgba(22, 163, 74, 0.12)", badge: "bg-emerald-500" },
  { stroke: "#db2777", fill: "rgba(219, 39, 119, 0.12)", badge: "bg-pink-500" },
];

const copyPoint = (point) => ({ ...point });
const clampNumber = (value, min, max) => Math.max(min, Math.min(max, value));

export const getZoneColor = (index) => ZONE_COLORS[index % ZONE_COLORS.length];

export const worldToCanvas = (x, y) => ({
  x: DRAWING_LEFT + (x + HALF_WIDTH) * SCALE,
  y: DRAWING_TOP + (HALF_HEIGHT - y) * SCALE,
});

export const canvasToWorld = (x, y) => ({
  x: (x - DRAWING_LEFT) / SCALE - HALF_WIDTH,
  y: HALF_HEIGHT - (y - DRAWING_TOP) / SCALE,
});

export const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

export const isInsideMap = (point) =>
  point.x >= -HALF_WIDTH &&
  point.x <= HALF_WIDTH &&
  point.y >= -HALF_HEIGHT &&
  point.y <= HALF_HEIGHT;

const clampPointToMap = (point) => ({
  x: clampNumber(point.x, -HALF_WIDTH + SAFE_POINT_MARGIN, HALF_WIDTH - SAFE_POINT_MARGIN),
  y: clampNumber(point.y, -HALF_HEIGHT + SAFE_POINT_MARGIN, HALF_HEIGHT - SAFE_POINT_MARGIN),
});

export const sanitizeRouteForController = (route) => {
  const cleaned = [];
  const minSegment = Math.max(CONTROLLER_MIN_SEGMENT, GRID_PATH_STEP * 0.55);

  for (const point of route) {
    const x = Number(point?.x);
    const y = Number(point?.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;

    const candidate = { x, y };
    if (!cleaned.length || dist(cleaned[cleaned.length - 1], candidate) >= minSegment) {
      cleaned.push(candidate);
    }
  }

  if (cleaned.length > 2) {
    const smoothed = [cleaned[0]];

    for (let i = 1; i < cleaned.length - 1; i += 1) {
      const prev = smoothed[smoothed.length - 1];
      const current = cleaned[i];
      const next = cleaned[i + 1];

      const firstLeg = dist(prev, current);
      const secondLeg = dist(current, next);
      const direct = dist(prev, next);
      const detour = firstLeg + secondLeg;
      const cross = Math.abs(
        (current.x - prev.x) * (next.y - current.y) -
        (current.y - prev.y) * (next.x - current.x)
      );

      const almostStraight = cross <= 0.02 && detour - direct <= 0.08;
      if (almostStraight) continue;
      smoothed.push(current);
    }

    smoothed.push(cleaned[cleaned.length - 1]);
    cleaned.length = 0;
    cleaned.push(...smoothed);
  }

  return cleaned;
};

const pointOnSegment = (point, a, b) => {
  const cross = (point.y - a.y) * (b.x - a.x) - (point.x - a.x) * (b.y - a.y);
  if (Math.abs(cross) > EPS) return false;
  const dot = (point.x - a.x) * (point.x - b.x) + (point.y - a.y) * (point.y - b.y);
  return dot <= EPS;
};

const getClosestPointOnSegment = (point, a, b) => {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const abLengthSquared = abx * abx + aby * aby;
  if (abLengthSquared <= EPS) return copyPoint(a);

  const t = clampNumber(
    ((point.x - a.x) * abx + (point.y - a.y) * aby) / abLengthSquared,
    0,
    1
  );

  return {
    x: a.x + abx * t,
    y: a.y + aby * t,
  };
};

const orientation = (a, b, c) => {
  const value = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
  if (Math.abs(value) <= EPS) return 0;
  return value > 0 ? 1 : 2;
};

export const pointEquals = (a, b) =>
  Math.abs(a.x - b.x) <= EPS && Math.abs(a.y - b.y) <= EPS;

export const segmentsIntersect = (a, b, c, d) => {
  const o1 = orientation(a, b, c);
  const o2 = orientation(a, b, d);
  const o3 = orientation(c, d, a);
  const o4 = orientation(c, d, b);

  if (o1 !== o2 && o3 !== o4) return true;
  if (o1 === 0 && pointOnSegment(c, a, b)) return true;
  if (o2 === 0 && pointOnSegment(d, a, b)) return true;
  if (o3 === 0 && pointOnSegment(a, c, d)) return true;
  if (o4 === 0 && pointOnSegment(b, c, d)) return true;
  return false;
};

export const pointInPolygon = (point, polygon) => {
  if (polygon.length < 3) return false;
  let inside = false;

  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i, i += 1) {
    const a = polygon[i];
    const b = polygon[j];
    if (pointOnSegment(point, a, b)) return true;

    const intersects =
      a.y > point.y !== b.y > point.y &&
      point.x < ((b.x - a.x) * (point.y - a.y)) / (b.y - a.y + EPS) + a.x;

    if (intersects) inside = !inside;
  }

  return inside;
};

export const pointInAnyPolygon = (point, polygons) =>
  polygons.some((polygon) => pointInPolygon(point, polygon.points));

const pointAtSegment = (a, b, t) => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
});

const distancePointToSegment = (point, a, b) =>
  dist(point, getClosestPointOnSegment(point, a, b));

const distancePointToPolygonEdges = (point, polygon) => {
  if (polygon.length < 2) return Number.POSITIVE_INFINITY;

  let best = Number.POSITIVE_INFINITY;
  for (let i = 0; i < polygon.length; i += 1) {
    const a = polygon[i];
    const b = polygon[(i + 1) % polygon.length];
    best = Math.min(best, distancePointToSegment(point, a, b));
  }

  return best;
};

const isPointNearPolygon = (point, polygon, margin = 0) => {
  if (pointInPolygon(point, polygon)) return true;
  if (margin <= EPS) return false;
  return distancePointToPolygonEdges(point, polygon) <= margin + EPS;
};

const findBlockingPolygon = (point, polygons, margin = 0) =>
  polygons.find((polygon) => isPointNearPolygon(point, polygon.points, margin));

const projectPointOutsidePolygon = (point, polygon, margin = SAFE_POINT_MARGIN) => {
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

    const normals = [
      { x: -edgeY / edgeLength, y: edgeX / edgeLength },
      { x: edgeY / edgeLength, y: -edgeX / edgeLength },
    ];

    const outsideCandidates = normals
      .map((normal) =>
        clampPointToMap({
          x: boundaryPoint.x + normal.x * targetMargin,
          y: boundaryPoint.y + normal.y * targetMargin,
        })
      )
      .filter((candidate) => !isPointNearPolygon(candidate, polygon, margin));

    if (!outsideCandidates.length) continue;

    const candidate = outsideCandidates.sort(
      (left, right) => dist(point, left) - dist(point, right)
    )[0];

    bestBoundaryPoint = boundaryPoint;
    bestCandidate = candidate;
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

  return {
    point: current,
    adjusted,
  };
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

const isInsideTraversableMap = (point, margin = MAP_TRAVERSAL_MARGIN) =>
  point.x >= -HALF_WIDTH + margin &&
  point.x <= HALF_WIDTH - margin &&
  point.y >= -HALF_HEIGHT + margin &&
  point.y <= HALF_HEIGHT - margin;

const segmentClear = (a, b, polygons, margin = ROUTE_CLEARANCE_MARGIN) => {
  if (!isInsideTraversableMap(a) || !isInsideTraversableMap(b)) return false;

  if (pointEquals(a, b)) {
    return !findBlockingPolygon(a, polygons, margin);
  }

  for (const polygon of polygons) {
    const points = polygon.points;
    if (
      isPointNearPolygon(a, points, Math.max(0, margin - 0.03)) ||
      isPointNearPolygon(b, points, Math.max(0, margin - 0.03))
    ) {
      return false;
    }

    for (const t of [0.25, 0.5, 0.75]) {
      if (pointInPolygon(pointAtSegment(a, b, t), points)) return false;
    }

    for (let i = 0; i < points.length; i += 1) {
      const p = points[i];
      const q = points[(i + 1) % points.length];
      if (distanceBetweenSegments(a, b, p, q) <= margin + EPS) {
        return false;
      }
    }
  }

  return true;
};

const routeCrossesPolygon = (route, polygon) => {
  if (route.length < 2 || polygon.points.length < 3) return false;

  for (let i = 1; i < route.length; i += 1) {
    if (!segmentClear(route[i - 1], route[i], [polygon], ROUTE_CLEARANCE_MARGIN)) {
      return true;
    }
  }

  return false;
};

export const routeCrossesAnyLimitPolygon = (route, polygons) =>
  polygons.some((polygon) => routeCrossesPolygon(route, polygon));

const buildGridIndex = (x, y, cols) => y * cols + x;

const createTraversalGrid = (polygons, margin = ROUTE_CLEARANCE_MARGIN) => {
  const originX = -HALF_WIDTH + MAP_TRAVERSAL_MARGIN;
  const originY = -HALF_HEIGHT + MAP_TRAVERSAL_MARGIN;
  const cols = Math.floor((MAP_WORLD_WIDTH - MAP_TRAVERSAL_MARGIN * 2) / GRID_PATH_STEP) + 1;
  const rows = Math.floor((MAP_WORLD_HEIGHT - MAP_TRAVERSAL_MARGIN * 2) / GRID_PATH_STEP) + 1;
  const points = Array(cols * rows);
  const blocked = Array(cols * rows).fill(false);

  for (let y = 0; y < rows; y += 1) {
    for (let x = 0; x < cols; x += 1) {
      const point = {
        x: originX + x * GRID_PATH_STEP,
        y: originY + y * GRID_PATH_STEP,
      };
      const index = buildGridIndex(x, y, cols);
      points[index] = point;
      blocked[index] =
        !isInsideMap(point) ||
        !isInsideTraversableMap(point) ||
        Boolean(findBlockingPolygon(point, polygons, margin));
    }
  }

  return { cols, rows, points, blocked };
};

const findNearestFreeGridCell = (point, grid, polygons, margin = ROUTE_CLEARANCE_MARGIN) => {
  const { cols, rows, points, blocked } = grid;
  const baseX = clampNumber(
    Math.round((point.x - (-HALF_WIDTH + MAP_TRAVERSAL_MARGIN)) / GRID_PATH_STEP),
    0,
    cols - 1
  );
  const baseY = clampNumber(
    Math.round((point.y - (-HALF_HEIGHT + MAP_TRAVERSAL_MARGIN)) / GRID_PATH_STEP),
    0,
    rows - 1
  );

  let bestIndex = -1;
  let bestDistance = Number.POSITIVE_INFINITY;

  for (let radius = 0; radius <= 12; radius += 1) {
    for (let y = Math.max(0, baseY - radius); y <= Math.min(rows - 1, baseY + radius); y += 1) {
      for (let x = Math.max(0, baseX - radius); x <= Math.min(cols - 1, baseX + radius); x += 1) {
        const index = buildGridIndex(x, y, cols);
        if (blocked[index]) continue;

        const candidate = points[index];
        if (!segmentClear(point, candidate, polygons, margin)) continue;

        const candidateDistance = dist(point, candidate);
        if (candidateDistance < bestDistance) {
          bestDistance = candidateDistance;
          bestIndex = index;
        }
      }
    }

    if (bestIndex >= 0) return bestIndex;
  }

  return -1;
};

const simplifyPathByVisibility = (path, polygons, margin = ROUTE_CLEARANCE_MARGIN) => {
  if (path.length <= 2) return path.map(copyPoint);

  const simplified = [copyPoint(path[0])];
  let anchor = 0;

  while (anchor < path.length - 1) {
    let next = path.length - 1;
    while (next > anchor + 1 && !segmentClear(path[anchor], path[next], polygons, margin)) {
      next -= 1;
    }
    simplified.push(copyPoint(path[next]));
    anchor = next;
  }

  return simplified;
};

const simplifyCollinearPath = (path) => {
  if (path.length <= 2) return path.map(copyPoint);

  const simplified = [copyPoint(path[0])];

  for (let i = 1; i < path.length - 1; i += 1) {
    const prev = simplified[simplified.length - 1];
    const current = path[i];
    const next = path[i + 1];

    const ax = current.x - prev.x;
    const ay = current.y - prev.y;
    const bx = next.x - current.x;
    const by = next.y - current.y;
    const cross = Math.abs(ax * by - ay * bx);

    if (cross <= COLLINEAR_EPS) continue;
    simplified.push(copyPoint(current));
  }

  simplified.push(copyPoint(path[path.length - 1]));
  return simplified;
};

const findShortestSafePath = (start, end, polygons, margin = ROUTE_CLEARANCE_MARGIN) => {
  if (segmentClear(start, end, polygons, margin)) {
    return [copyPoint(start), copyPoint(end)];
  }

  const grid = createTraversalGrid(polygons, margin);
  const startIndex = findNearestFreeGridCell(start, grid, polygons, margin);
  const endIndex = findNearestFreeGridCell(end, grid, polygons, margin);

  if (startIndex < 0 || endIndex < 0) return null;
  if (startIndex === endIndex) {
    const anchor = grid.points[startIndex];
    const candidate = [copyPoint(start), copyPoint(anchor), copyPoint(end)];
    return simplifyPathByVisibility(candidate, polygons, margin);
  }

  const { cols, rows, points, blocked } = grid;
  const total = points.length;
  const open = new Set([startIndex]);
  const closed = Array(total).fill(false);
  const gScore = Array(total).fill(Number.POSITIVE_INFINITY);
  const fScore = Array(total).fill(Number.POSITIVE_INFINITY);
  const previous = Array(total).fill(-1);

  gScore[startIndex] = 0;
  fScore[startIndex] = dist(points[startIndex], points[endIndex]);

  const directions = [
    [-1, -1], [0, -1], [1, -1],
    [-1, 0],            [1, 0],
    [-1, 1],  [0, 1],   [1, 1],
  ];

  while (open.size) {
    let current = -1;
    let bestF = Number.POSITIVE_INFINITY;

    for (const index of open) {
      if (fScore[index] < bestF) {
        bestF = fScore[index];
        current = index;
      }
    }

    if (current < 0) break;
    if (current === endIndex) break;

    open.delete(current);
    closed[current] = true;

    const currentX = current % cols;
    const currentY = Math.floor(current / cols);

    for (const [dx, dy] of directions) {
      const nextX = currentX + dx;
      const nextY = currentY + dy;
      if (nextX < 0 || nextX >= cols || nextY < 0 || nextY >= rows) continue;

      const nextIndex = buildGridIndex(nextX, nextY, cols);
      if (closed[nextIndex] || blocked[nextIndex]) continue;

      const fromPoint = points[current];
      const toPoint = points[nextIndex];
      if (!segmentClear(fromPoint, toPoint, polygons, margin)) continue;

      const tentative = gScore[current] + dist(fromPoint, toPoint);
      if (tentative + EPS >= gScore[nextIndex]) continue;

      previous[nextIndex] = current;
      gScore[nextIndex] = tentative;
      fScore[nextIndex] = tentative + dist(toPoint, points[endIndex]);
      open.add(nextIndex);
    }
  }

  if (!Number.isFinite(gScore[endIndex])) return null;

  const path = [];
  for (let current = endIndex; current !== -1; current = previous[current]) {
    path.push(copyPoint(points[current]));
  }

  const rawPath = [copyPoint(start), ...path.reverse(), copyPoint(end)];
  return simplifyCollinearPath(simplifyPathByVisibility(rawPath, polygons, margin));
};

export const buildObstacleAwareRoute = (route, polygons) => {
  if (route.length <= 1 || polygons.length === 0) return route.map(copyPoint);

  const result = [];
  for (let i = 0; i < route.length - 1; i += 1) {
    const path = findShortestSafePath(route[i], route[i + 1], polygons, ROUTE_CLEARANCE_MARGIN);
    if (!path) return null;

    if (!result.length) result.push(...path);
    else result.push(...path.slice(1));
  }

  return simplifyCollinearPath(result);
};

export const drawDiamond = (ctx, x, y, radius) => {
  ctx.beginPath();
  ctx.moveTo(x, y - radius);
  ctx.lineTo(x + radius, y);
  ctx.lineTo(x, y + radius);
  ctx.lineTo(x - radius, y);
  ctx.closePath();
};

export const drawPlannerBackground = (ctx) => {
  const gradient = ctx.createLinearGradient(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
  gradient.addColorStop(0, "#fafafa");
  gradient.addColorStop(1, "#e5e7eb");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  ctx.fillStyle = "#ffffff";
  ctx.fillRect(DRAWING_LEFT, DRAWING_TOP, DRAWING_WIDTH, DRAWING_HEIGHT);
  ctx.strokeStyle = "#0f172a";
  ctx.lineWidth = 2;
  ctx.strokeRect(DRAWING_LEFT, DRAWING_TOP, DRAWING_WIDTH, DRAWING_HEIGHT);

  ctx.strokeStyle = "rgba(148, 163, 184, 0.18)";
  ctx.lineWidth = 1;
  for (let x = -HALF_WIDTH; x <= HALF_WIDTH; x += 1) {
    const from = worldToCanvas(x, -HALF_HEIGHT);
    const to = worldToCanvas(x, HALF_HEIGHT);
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.stroke();
  }

  for (let y = -HALF_HEIGHT; y <= HALF_HEIGHT; y += 1) {
    const from = worldToCanvas(-HALF_WIDTH, y);
    const to = worldToCanvas(HALF_WIDTH, y);
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(100, 116, 139, 0.28)";
  ctx.lineWidth = 1.5;
  for (let x = -HALF_WIDTH; x <= HALF_WIDTH; x += 4) {
    const from = worldToCanvas(x, -HALF_HEIGHT);
    const to = worldToCanvas(x, HALF_HEIGHT);
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.stroke();
  }

  for (let y = -HALF_HEIGHT; y <= HALF_HEIGHT; y += 4) {
    const from = worldToCanvas(-HALF_WIDTH, y);
    const to = worldToCanvas(HALF_WIDTH, y);
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.stroke();
  }

  const verticalAxisTop = worldToCanvas(0, HALF_HEIGHT);
  const verticalAxisBottom = worldToCanvas(0, -HALF_HEIGHT);
  const horizontalAxisLeft = worldToCanvas(-HALF_WIDTH, 0);
  const horizontalAxisRight = worldToCanvas(HALF_WIDTH, 0);

  ctx.strokeStyle = "rgba(15, 23, 42, 0.6)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(verticalAxisTop.x, verticalAxisTop.y);
  ctx.lineTo(verticalAxisBottom.x, verticalAxisBottom.y);
  ctx.moveTo(horizontalAxisLeft.x, horizontalAxisLeft.y);
  ctx.lineTo(horizontalAxisRight.x, horizontalAxisRight.y);
  ctx.stroke();

  ctx.fillStyle = "#334155";
  ctx.font = "600 12px 'Segoe UI', sans-serif";
  ctx.fillText("Y", verticalAxisTop.x + 8, verticalAxisTop.y + 18);
  ctx.fillText("X", horizontalAxisRight.x - 18, horizontalAxisRight.y - 8);
  ctx.fillText("Neutral routing grid", DRAWING_LEFT + 14, DRAWING_TOP - 14);
};
