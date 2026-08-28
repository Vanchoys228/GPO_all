import { buildObstacleAwareRoute, dist } from "../../../lib/zonePlanner";
import { estimateRouteEnergy } from "../../../lib/energyModel";
import {
  EPS,
  copyPoint,
  dedupeConsecutiveRoute,
  directedEdgeKey,
  distancePointToSegment,
  mergeRoute,
  polylineLength,
  samePoint,
} from "./chargingPlannerGeometry";
import { dominates } from "./chargingPlannerFrontier";

const COST_EPS = 1e-6;
const FUEL_EPS = 1e-6;
const STATION_DETOUR_WEIGHT = 0.85;
const STATION_EXTRA_PATH_WEIGHT = 0.55;
const ENERGY_SCORE_WEIGHT = 0.22;

const getSafeSegment = (from, to, polygons, cache) => {
  if (samePoint(from, to)) return { route: [copyPoint(from)], distance: 0 };
  const key = directedEdgeKey(from, to);
  const cached = cache.get(key);
  if (cached) return cached;

  const path = buildObstacleAwareRoute([from, to], polygons);
  if (!path || path.length < 2) return null;
  const route = dedupeConsecutiveRoute(path);
  const distance = polylineLength(route);
  if (!Number.isFinite(distance)) return null;

  const direct = { route, distance };
  cache.set(key, direct);
  cache.set(directedEdgeKey(to, from), { route: [...route].reverse().map(copyPoint), distance });
  return direct;
};

const buildLegGraph = ({
  start, end, stations, polygons, batteryRange, surfaceZones, energyOptions, segmentCache,
}) => {
  const nodes = [copyPoint(start), copyPoint(end), ...stations.map(copyPoint)];
  const adjacency = Array.from({ length: nodes.length }, () => []);
  for (let from = 0; from < nodes.length; from += 1) {
    for (let to = 0; to < nodes.length; to += 1) {
      if (from === to) continue;
      const segment = getSafeSegment(nodes[from], nodes[to], polygons, segmentCache);
      if (!segment) continue;
      const energy = estimateRouteEnergy(segment.route, { surfaceZones, ...energyOptions }).totalEnergy;
      if (!Number.isFinite(energy) || energy > batteryRange + EPS) continue;
      adjacency[from].push({ to, distance: segment.distance, energy, route: segment.route });
    }
  }
  return { nodes, adjacency };
};

const reconstructLegRoute = (labels, targetLabelIndex) => {
  const chain = [];
  for (let cursor = targetLabelIndex; cursor >= 0; cursor = labels[cursor].parent) chain.push(labels[cursor]);
  chain.reverse();
  let route = [];
  let stationStops = 0;
  for (const label of chain) {
    if (!label.edgeRoute) route = [copyPoint(label.point)];
    else route = mergeRoute(route, label.edgeRoute);
    if (label.arrivedAtStation) stationStops += 1;
  }
  return { route: dedupeConsecutiveRoute(route), stationStops };
};

export const findLegFrontier = ({
  start, end, startFuel, stations, polygons, batteryRange, surfaceZones, energyOptions, segmentCache,
}) => {
  const { nodes, adjacency } = buildLegGraph({
    start, end, stations, polygons, batteryRange, surfaceZones, energyOptions, segmentCache,
  });
  const labels = [];
  const frontiers = Array.from({ length: nodes.length }, () => []);
  const open = [];
  const enqueueLabel = (candidate) => {
    const current = frontiers[candidate.node];
    if (current.some((index) => dominates(labels[index], candidate))) return;
    frontiers[candidate.node] = current.filter((index) => !dominates(candidate, labels[index]));
    const index = labels.length;
    labels.push(candidate);
    frontiers[candidate.node].push(index);
    open.push(index);
  };

  enqueueLabel({
    node: 0, point: nodes[0], cost: 0, distance: 0, energy: 0, fuel: startFuel,
    parent: -1, edgeRoute: null, arrivedAtStation: false,
  });
  while (open.length) {
    let bestOpenIndex = 0;
    for (let index = 1; index < open.length; index += 1) {
      const left = labels[open[index]];
      const right = labels[open[bestOpenIndex]];
      if (left.cost < right.cost - COST_EPS || (Math.abs(left.cost - right.cost) <= COST_EPS && left.fuel > right.fuel + FUEL_EPS)) bestOpenIndex = index;
    }
    const labelIndex = open.splice(bestOpenIndex, 1)[0];
    const label = labels[labelIndex];
    for (const edge of adjacency[label.node]) {
      if (edge.energy > label.fuel + FUEL_EPS) continue;
      const rawFuel = label.fuel - edge.energy;
      if (rawFuel < -FUEL_EPS) continue;
      const stationNode = edge.to >= 2;
      const stationPoint = nodes[edge.to];
      const stationLineOffset = distancePointToSegment(stationPoint, start, end);
      const stationExtraPath = Math.max(0, dist(start, stationPoint) + dist(stationPoint, end) - dist(start, end));
      const stationDetourPenalty = stationNode
        ? stationLineOffset * STATION_DETOUR_WEIGHT + stationExtraPath * STATION_EXTRA_PATH_WEIGHT
        : 0;
      enqueueLabel({
        node: edge.to,
        point: stationPoint,
        cost: label.cost + edge.distance + edge.energy * ENERGY_SCORE_WEIGHT + stationDetourPenalty,
        distance: label.distance + edge.distance,
        energy: label.energy + edge.energy,
        fuel: Math.max(0, stationNode ? batteryRange : rawFuel),
        parent: labelIndex,
        edgeRoute: edge.route,
        arrivedAtStation: stationNode,
      });
    }
  }

  return frontiers[1]
    .map((index) => ({ index, ...labels[index] }))
    .sort((left, right) => left.cost - right.cost || left.distance - right.distance || left.energy - right.energy || right.fuel - left.fuel)
    .map((entry) => {
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
