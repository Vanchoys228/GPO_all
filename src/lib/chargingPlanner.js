import { buildObstacleAwareRoute, dist } from "./zonePlanner";
import { DEFAULT_ENERGY_OPTIONS, estimateRouteEnergy } from "./energyModel";

export const DEFAULT_BATTERY_RANGE_METERS = 100;
const EPS = 1e-6;
const COST_EPS = 1e-6;
const FUEL_EPS = 1e-6;
const FRONTIER_LIMIT = 28;
const POINT_KEY_DIGITS = 4;
const STATION_DETOUR_WEIGHT = 0.85;
const STATION_EXTRA_PATH_WEIGHT = 0.55;
const ENERGY_SCORE_WEIGHT = 0.22;

const toPoint = (point) => ({
  x: Number(point?.x),
  y: Number(point?.y),
});

const isFinitePoint = (point) =>
  Number.isFinite(point?.x) && Number.isFinite(point?.y);

const copyPoint = (point) => ({ x: point.x, y: point.y });

const samePoint = (left, right) =>
  Math.abs(left.x - right.x) <= EPS && Math.abs(left.y - right.y) <= EPS;

const pointKey = (point) =>
  `${Number(point.x).toFixed(POINT_KEY_DIGITS)},${Number(point.y).toFixed(POINT_KEY_DIGITS)}`;

const directedEdgeKey = (from, to) => `${pointKey(from)}->${pointKey(to)}`;

const polylineLength = (route) =>
  route.reduce((sum, point, index) => (index ? sum + dist(route[index - 1], point) : 0), 0);

const distancePointToSegment = (point, start, end) => {
  const abx = end.x - start.x;
  const aby = end.y - start.y;
  const denominator = abx * abx + aby * aby;
  if (denominator <= EPS) return dist(point, start);

  const t = Math.max(
    0,
    Math.min(
      1,
      ((point.x - start.x) * abx + (point.y - start.y) * aby) / denominator
    )
  );
  const closest = {
    x: start.x + abx * t,
    y: start.y + aby * t,
  };
  return dist(point, closest);
};

const dedupeConsecutiveRoute = (route) => {
  if (!route.length) return [];
  const cleaned = [copyPoint(route[0])];
  for (let index = 1; index < route.length; index += 1) {
    if (!samePoint(cleaned[cleaned.length - 1], route[index])) {
      cleaned.push(copyPoint(route[index]));
    }
  }
  return cleaned;
};

const mergeRoute = (left, right) => {
  if (!left.length) return right.map(copyPoint);
  if (!right.length) return left.map(copyPoint);

  if (samePoint(left[left.length - 1], right[0])) {
    return left.concat(right.slice(1).map(copyPoint));
  }
  return left.concat(right.map(copyPoint));
};

const normalizeMandatoryRoute = (route) =>
  dedupeConsecutiveRoute(
    (route || []).map(toPoint).filter(isFinitePoint)
  );

const normalizeStations = (stations) => {
  const unique = new Map();
  for (const raw of stations || []) {
    const point = toPoint(raw);
    if (!isFinitePoint(point)) continue;
    unique.set(pointKey(point), point);
  }
  return Array.from(unique.values());
};

const clampBatteryRange = (batteryRange) => {
  const value = Number(batteryRange);
  if (!Number.isFinite(value) || value <= 0) return null;
  return Math.max(0.1, value);
};

const dominates = (left, right) =>
  left.cost <= right.cost + COST_EPS && left.fuel >= right.fuel - FUEL_EPS;

const pruneDominatedLabels = (labels) => {
  const sorted = [...labels].sort((left, right) =>
    left.cost === right.cost ? right.fuel - left.fuel : left.cost - right.cost
  );
  const pruned = [];
  for (const label of sorted) {
    if (pruned.some((candidate) => dominates(candidate, label))) continue;
    pruned.push(label);
  }
  return pruned;
};

const keepFrontierBounded = (labels, limit = FRONTIER_LIMIT) => {
  const pruned = pruneDominatedLabels(labels);
  if (pruned.length <= limit) return pruned;
  return pruned
    .sort((left, right) => {
      if (left.cost !== right.cost) return left.cost - right.cost;
      if (left.fuel !== right.fuel) return right.fuel - left.fuel;
      return left.stationStops - right.stationStops;
    })
    .slice(0, limit);
};

const getSafeSegment = (from, to, polygons, cache) => {
  if (samePoint(from, to)) {
    return {
      route: [copyPoint(from)],
      distance: 0,
    };
  }

  const directKey = directedEdgeKey(from, to);
  const cached = cache.get(directKey);
  if (cached) return cached;

  const path = buildObstacleAwareRoute([from, to], polygons);
  if (!path || path.length < 2) return null;

  const route = dedupeConsecutiveRoute(path);
  const distance = polylineLength(route);
  if (!Number.isFinite(distance)) return null;

  const direct = { route, distance };
  const reverse = {
    route: [...route].reverse().map(copyPoint),
    distance,
  };
  cache.set(directKey, direct);
  cache.set(directedEdgeKey(to, from), reverse);
  return direct;
};

const buildLegGraph = ({
  start,
  end,
  stations,
  polygons,
  batteryRange,
  surfaceZones,
  energyOptions,
  segmentCache,
}) => {
  const nodes = [copyPoint(start), copyPoint(end), ...stations.map(copyPoint)];
  const isStationNode = (index) => index >= 2;
  const adjacency = Array.from({ length: nodes.length }, () => []);

  for (let from = 0; from < nodes.length; from += 1) {
    for (let to = 0; to < nodes.length; to += 1) {
      if (from === to) continue;

      const segment = getSafeSegment(nodes[from], nodes[to], polygons, segmentCache);
      if (!segment) continue;

      const metrics = estimateRouteEnergy(segment.route, {
        surfaceZones,
        ...energyOptions,
      });
      const energy = metrics.totalEnergy;
      if (!Number.isFinite(energy)) continue;
      if (energy > batteryRange + EPS) continue;

      adjacency[from].push({
        to,
        distance: segment.distance,
        energy,
        route: segment.route,
      });
    }
  }

  return { nodes, adjacency, isStationNode };
};

const reconstructLegRoute = (labels, targetLabelIndex) => {
  const chain = [];
  for (let cursor = targetLabelIndex; cursor >= 0; cursor = labels[cursor].parent) {
    chain.push(labels[cursor]);
  }
  chain.reverse();

  let route = [];
  let stationStops = 0;
  for (let index = 0; index < chain.length; index += 1) {
    const label = chain[index];
    if (!label.edgeRoute) {
      route = [copyPoint(label.point)];
      continue;
    }
    route = mergeRoute(route, label.edgeRoute);
    if (label.arrivedAtStation) stationStops += 1;
  }

  return {
    route: dedupeConsecutiveRoute(route),
    stationStops,
  };
};

const findLegFrontier = ({
  start,
  end,
  startFuel,
  stations,
  polygons,
  batteryRange,
  surfaceZones,
  energyOptions,
  segmentCache,
}) => {
  const graph = buildLegGraph({
    start,
    end,
    stations,
    polygons,
    batteryRange,
    surfaceZones,
    energyOptions,
    segmentCache,
  });
  const { nodes, adjacency, isStationNode } = graph;
  const targetNode = 1;

  const labels = [];
  const frontiers = Array.from({ length: nodes.length }, () => []);
  const open = [];

  const enqueueLabel = (candidate) => {
    const current = frontiers[candidate.node];
    for (const index of current) {
      if (dominates(labels[index], candidate)) return -1;
    }

    const filtered = current.filter((index) => !dominates(candidate, labels[index]));
    frontiers[candidate.node] = filtered;

    const nextIndex = labels.length;
    labels.push(candidate);
    frontiers[candidate.node].push(nextIndex);
    open.push(nextIndex);
    return nextIndex;
  };

  enqueueLabel({
    node: 0,
    point: nodes[0],
    cost: 0,
    distance: 0,
    energy: 0,
    fuel: startFuel,
    parent: -1,
    edgeRoute: null,
    arrivedAtStation: false,
  });

  while (open.length) {
    let bestOpenIndex = 0;
    for (let i = 1; i < open.length; i += 1) {
      const left = labels[open[i]];
      const right = labels[open[bestOpenIndex]];
      if (left.cost < right.cost - COST_EPS) {
        bestOpenIndex = i;
        continue;
      }
      if (Math.abs(left.cost - right.cost) <= COST_EPS && left.fuel > right.fuel + FUEL_EPS) {
        bestOpenIndex = i;
      }
    }

    const labelIndex = open.splice(bestOpenIndex, 1)[0];
    const label = labels[labelIndex];

    for (const edge of adjacency[label.node]) {
      if (edge.energy > label.fuel + FUEL_EPS) continue;

      const node = edge.to;
      const rawFuel = label.fuel - edge.energy;
      if (rawFuel < -FUEL_EPS) continue;

      const stationPoint = nodes[node];
      const stationLineOffset = distancePointToSegment(stationPoint, start, end);
      const stationExtraPath = Math.max(
        0,
        dist(start, stationPoint) + dist(stationPoint, end) - dist(start, end)
      );
      const stationDetourPenalty = isStationNode(node)
        ? stationLineOffset * STATION_DETOUR_WEIGHT +
          stationExtraPath * STATION_EXTRA_PATH_WEIGHT
        : 0;

      const fuel = isStationNode(node) ? batteryRange : rawFuel;
      enqueueLabel({
        node,
        point: stationPoint,
        cost: label.cost + edge.distance + edge.energy * ENERGY_SCORE_WEIGHT + stationDetourPenalty,
        distance: label.distance + edge.distance,
        energy: label.energy + edge.energy,
        fuel: Math.max(0, fuel),
        parent: labelIndex,
        edgeRoute: edge.route,
        arrivedAtStation: isStationNode(node),
      });
    }
  }

  const targetLabels = frontiers[targetNode]
    .map((index) => ({
      index,
      cost: labels[index].cost,
      distance: labels[index].distance,
      energy: labels[index].energy,
      fuel: labels[index].fuel,
    }))
    .sort((left, right) => {
      if (left.cost !== right.cost) return left.cost - right.cost;
      if (left.distance !== right.distance) return left.distance - right.distance;
      if (left.energy !== right.energy) return left.energy - right.energy;
      return right.fuel - left.fuel;
    });

  return targetLabels.map((entry) => {
    const reconstructed = reconstructLegRoute(labels, entry.index);
    return {
      score: entry.cost,
      distance: entry.distance,
      energy: entry.energy,
      fuelEnd: entry.fuel,
      route: reconstructed.route,
      stationStops: reconstructed.stationStops,
    };
  });
};

export const planRouteWithCharging = ({
  route,
  stations = [],
  polygons = [],
  surfaceZones = [],
  energyOptions = DEFAULT_ENERGY_OPTIONS,
  batteryRange = DEFAULT_BATTERY_RANGE_METERS,
}) => {
  const mandatory = normalizeMandatoryRoute(route);
  if (mandatory.length < 2) {
    const metrics = estimateRouteEnergy(mandatory, {
      surfaceZones,
      ...energyOptions,
    });
    return {
      ok: true,
      route: mandatory,
      routeDistance: polylineLength(mandatory),
      routeEnergy: metrics.totalEnergy,
      stationStopCount: 0,
    };
  }

  const normalizedBatteryRange = clampBatteryRange(batteryRange);
  if (!normalizedBatteryRange) {
    return {
      ok: false,
      reason: "invalid_battery_range",
      error: "Запас хода должен быть больше нуля.",
    };
  }

  const normalizedStations = normalizeStations(stations);
  const segmentCache = new Map();

  let frontier = [
    {
      cost: 0,
      distance: 0,
      energySpent: 0,
      fuel: normalizedBatteryRange,
      route: [copyPoint(mandatory[0])],
      stationStops: 0,
    },
  ];

  for (let legIndex = 0; legIndex < mandatory.length - 1; legIndex += 1) {
    const nextFrontier = [];
    const legStart = mandatory[legIndex];
    const legEnd = mandatory[legIndex + 1];

    for (const label of frontier) {
      const legOptions = findLegFrontier({
        start: legStart,
        end: legEnd,
        startFuel: label.fuel,
        stations: normalizedStations,
        polygons,
        batteryRange: normalizedBatteryRange,
        surfaceZones,
        energyOptions,
        segmentCache,
      });

      for (const option of legOptions) {
        nextFrontier.push({
          cost: label.cost + option.score,
          distance: label.distance + option.distance,
          energySpent: label.energySpent + option.energy,
          fuel: option.fuelEnd,
          route: mergeRoute(label.route, option.route),
          stationStops: label.stationStops + option.stationStops,
        });
      }
    }

    frontier = keepFrontierBounded(nextFrontier);
    if (!frontier.length) {
      return {
        ok: false,
        reason: "insufficient_range",
        error:
          "Маршрут недостижим при текущем запасе хода: добавьте станции зарядки или увеличьте запас.",
        blockedLegIndex: legIndex,
      };
    }
  }

  const best = frontier.sort((left, right) => {
    if (left.cost !== right.cost) return left.cost - right.cost;
    if (left.distance !== right.distance) return left.distance - right.distance;
    if (left.energySpent !== right.energySpent) return left.energySpent - right.energySpent;
    if (left.stationStops !== right.stationStops) return left.stationStops - right.stationStops;
    if (left.fuel !== right.fuel) return right.fuel - left.fuel;
    return 0;
  })[0];

  const normalizedRoute = dedupeConsecutiveRoute(best.route);
  const routeMetrics = estimateRouteEnergy(normalizedRoute, {
    surfaceZones,
    ...energyOptions,
  });
  return {
    ok: true,
    route: normalizedRoute,
    routeDistance: polylineLength(normalizedRoute),
    routeEnergy: routeMetrics.totalEnergy,
    estimatedTimeSec: routeMetrics.estimatedTimeSec,
    limitingMaxSpeedMps: routeMetrics.limitingMaxSpeedMps,
    averageSlipRisk: routeMetrics.averageSlipRisk,
    stationStopCount: best.stationStops,
    batteryRange: normalizedBatteryRange,
  };
};
