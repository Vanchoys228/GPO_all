import { DEFAULT_ENERGY_OPTIONS, estimateRouteEnergy } from "./energyModel";
import {
  clampBatteryRange,
  copyPoint,
  dedupeConsecutiveRoute,
  mergeRoute,
  normalizeMandatoryRoute,
  normalizeStations,
  polylineLength,
} from "../features/planner/model/chargingPlannerGeometry";
import { keepFrontierBounded } from "../features/planner/model/chargingPlannerFrontier";
import { findLegFrontier } from "../features/planner/model/chargingPlannerLegs";

export const DEFAULT_BATTERY_RANGE_METERS = 100;

const selectBestRoute = (frontier) =>
  [...frontier].sort((left, right) => {
    if (left.cost !== right.cost) return left.cost - right.cost;
    if (left.distance !== right.distance) return left.distance - right.distance;
    if (left.energySpent !== right.energySpent) return left.energySpent - right.energySpent;
    if (left.stationStops !== right.stationStops) return left.stationStops - right.stationStops;
    return right.fuel - left.fuel;
  })[0];

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
    const metrics = estimateRouteEnergy(mandatory, { surfaceZones, ...energyOptions });
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
  let frontier = [{
    cost: 0,
    distance: 0,
    energySpent: 0,
    fuel: normalizedBatteryRange,
    route: [copyPoint(mandatory[0])],
    stationStops: 0,
  }];

  for (let legIndex = 0; legIndex < mandatory.length - 1; legIndex += 1) {
    const nextFrontier = [];
    for (const label of frontier) {
      const legOptions = findLegFrontier({
        start: mandatory[legIndex],
        end: mandatory[legIndex + 1],
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
        error: "Маршрут недостижим при текущем запасе хода: добавьте станции зарядки или увеличьте запас.",
        blockedLegIndex: legIndex,
      };
    }
  }

  const best = selectBestRoute(frontier);
  const normalizedRoute = dedupeConsecutiveRoute(best.route);
  const routeMetrics = estimateRouteEnergy(normalizedRoute, { surfaceZones, ...energyOptions });
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
