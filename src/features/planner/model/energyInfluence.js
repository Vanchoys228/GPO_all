import { DEFAULT_PAYLOAD_KG, DEFAULT_SPEED_MPS, SURFACE_ZONE_PRESETS } from "./energyProfiles";
import { estimateRouteEnergy } from "./energyEstimator";

const safeNumber = (value, fallback) => Number.isFinite(Number(value)) ? Number(value) : fallback;
const formatSigned = (value, digits = 1) => {
  if (!Number.isFinite(value) || Math.abs(value) < 0.05) return "0";
  return `${value > 0 ? "+" : "-"}${Math.abs(value).toFixed(digits)}`;
};
const formatEnergyImpact = (value, totalEnergy) => {
  const percent = (value / Math.max(Math.abs(totalEnergy), 1)) * 100;
  return `${formatSigned(value, 1)} ед. (${formatSigned(percent, 1)}%)`;
};
const countClosedSurfaceZones = (surfaceZones) =>
  (Array.isArray(surfaceZones) ? surfaceZones : []).filter((zone) => zone?.closed !== false && Array.isArray(zone?.points) && zone.points.length >= 3).length;

export const analyzeRouteInfluence = (route, {
  surfaceZones = SURFACE_ZONE_PRESETS,
  speedMps = DEFAULT_SPEED_MPS,
  payloadKg = DEFAULT_PAYLOAD_KG,
  stationStopCount = 0,
  plannedTimeSec = 0,
  actualTimeSec = null,
  avoidanceTimeSec = 0,
} = {}) => {
  if (!Array.isArray(route) || route.length < 2) return [];
  const actual = estimateRouteEnergy(route, { surfaceZones, speedMps, payloadKg });
  const neutralSameInputs = estimateRouteEnergy(route, { surfaceZones: [], speedMps, payloadKg });
  const noPayload = estimateRouteEnergy(route, { surfaceZones, speedMps, payloadKg: 0 });
  const defaultSpeed = estimateRouteEnergy(route, { surfaceZones, speedMps: DEFAULT_SPEED_MPS, payloadKg });
  const noTurns = estimateRouteEnergy(route, { surfaceZones, speedMps, payloadKg, includeTurnPenalty: false });
  const neutralBase = estimateRouteEnergy(route, { surfaceZones: [], speedMps: DEFAULT_SPEED_MPS, payloadKg: 0, includeTurnPenalty: false });
  const rows = [
    { key: "distance", label: "Длина маршрута", value: `${actual.distanceMeters.toFixed(1)} м`, impact: `${neutralBase.totalEnergy.toFixed(1)} ед. базового расхода` },
    { key: "surfaces", label: "Типы покрытий", value: `${countClosedSurfaceZones(surfaceZones)} зон`, impact: formatEnergyImpact(actual.totalEnergy - neutralSameInputs.totalEnergy, actual.totalEnergy) },
    { key: "payload", label: "Масса груза", value: `${Math.max(0, safeNumber(payloadKg, 0)).toFixed(1)} кг`, impact: formatEnergyImpact(actual.totalEnergy - noPayload.totalEnergy, actual.totalEnergy) },
    { key: "speed", label: "Заданная скорость", value: `${Math.max(0, safeNumber(speedMps, DEFAULT_SPEED_MPS)).toFixed(2)} м/с`, impact: formatEnergyImpact(actual.totalEnergy - defaultSpeed.totalEnergy, actual.totalEnergy) },
    { key: "turns", label: "Повороты и манёвры", value: `${Math.max(0, route.length - 2)} поворотов`, impact: formatEnergyImpact(actual.totalEnergy - noTurns.totalEnergy, actual.totalEnergy) },
    { key: "slip", label: "Риск проскальзывания", value: `${(actual.averageSlipRisk * 100).toFixed(1)}%`, impact: actual.averageSlipRisk > 0.16 ? "нужна сниженная скорость" : actual.averageSlipRisk > 0.07 ? "умеренное влияние" : "низкое влияние" },
    { key: "charging", label: "Заезды на зарядку", value: `${Math.max(0, Number(stationStopCount) || 0)}`, impact: Number(stationStopCount) > 0 ? "маршрут удлинён зарядкой" : "без влияния" },
    { key: "avoidance-time", label: "Объезд вне маршрута", value: `${Math.round(Math.max(0, Number(avoidanceTimeSec) || 0))} сек`, impact: Number(avoidanceTimeSec) > 0 ? `+${Math.round(Number(avoidanceTimeSec))} сек к факту` : "без внепланового объезда" },
  ];
  if (Number.isFinite(actualTimeSec) && actualTimeSec > 0 && Number.isFinite(plannedTimeSec)) {
    rows.push({ key: "actual-time", label: "Факт против плана", value: `${Math.round(actualTimeSec)} сек`, impact: `${formatSigned(actualTimeSec - plannedTimeSec, 0)} сек` });
  }
  return rows;
};
