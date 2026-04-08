const EPS = 1e-9;
const DEFAULT_SPEED_MPS = 0.22;
const DEFAULT_PAYLOAD_KG = 0;

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

const normalizeAngle = (angle) => {
  let normalized = angle;
  while (normalized > Math.PI) normalized -= Math.PI * 2;
  while (normalized < -Math.PI) normalized += Math.PI * 2;
  return normalized;
};

const safeNumber = (value, fallback) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

const pointOnSegment = (point, a, b) => {
  const cross = (point.y - a.y) * (b.x - a.x) - (point.x - a.x) * (b.y - a.y);
  if (Math.abs(cross) > EPS) return false;
  const dot = (point.x - a.x) * (point.x - b.x) + (point.y - a.y) * (point.y - b.y);
  return dot <= EPS;
};

const pointInPolygon = (point, polygon) => {
  if (!Array.isArray(polygon) || polygon.length < 3) return false;
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

export const SURFACE_PROFILES = {
  neutral: {
    key: "neutral",
    label: "Нейтральное покрытие",
    fill: "rgba(148, 163, 184, 0.08)",
    stroke: "rgba(71, 85, 105, 0.35)",
    energyPerMeter: 1.0,
    maxSpeedMps: 0.24,
    nominalSpeedMps: 0.22,
    speedPenaltyGain: 0.4,
    payloadPenaltyPerKg: 0.012,
    turnEnergyPerRad: 0.32,
    slipRisk: 0.03,
  },
  rough: {
    key: "rough",
    label: "Шероховатое покрытие",
    fill: "rgba(245, 158, 11, 0.12)",
    stroke: "rgba(180, 83, 9, 0.4)",
    energyPerMeter: 1.32,
    maxSpeedMps: 0.18,
    nominalSpeedMps: 0.17,
    speedPenaltyGain: 0.72,
    payloadPenaltyPerKg: 0.023,
    turnEnergyPerRad: 0.46,
    slipRisk: 0.08,
  },
  slippery: {
    key: "slippery",
    label: "Скользкое покрытие",
    fill: "rgba(56, 189, 248, 0.12)",
    stroke: "rgba(2, 132, 199, 0.38)",
    energyPerMeter: 1.12,
    maxSpeedMps: 0.16,
    nominalSpeedMps: 0.15,
    speedPenaltyGain: 0.95,
    payloadPenaltyPerKg: 0.017,
    turnEnergyPerRad: 0.74,
    slipRisk: 0.24,
  },
};

export const SURFACE_PROFILE_OPTIONS = Object.values(SURFACE_PROFILES);

export const SURFACE_ZONE_PRESETS = [
  {
    id: "surface-rough-west",
    surfaceKey: "rough",
    name: "Западный шероховатый участок",
    points: [
      { x: -22, y: -17 },
      { x: -6, y: -17 },
      { x: -6, y: -4 },
      { x: -22, y: -4 },
    ],
  },
  {
    id: "surface-rough-center",
    surfaceKey: "rough",
    name: "Центральная зона стыков",
    points: [
      { x: -3.8, y: -17 },
      { x: 3.8, y: -17 },
      { x: 3.8, y: 17 },
      { x: -3.8, y: 17 },
    ],
  },
  {
    id: "surface-slippery-east",
    surfaceKey: "slippery",
    name: "Восточный скользкий участок",
    points: [
      { x: 8, y: 4.5 },
      { x: 22, y: 4.5 },
      { x: 22, y: 17 },
      { x: 8, y: 17 },
    ],
  },
];

export const DEFAULT_ENERGY_OPTIONS = {
  speedMps: DEFAULT_SPEED_MPS,
  payloadKg: DEFAULT_PAYLOAD_KG,
};

export const getSurfaceProfileByKey = (key) => SURFACE_PROFILES[key] || SURFACE_PROFILES.neutral;

const normalizeSurfaceZones = (zones) =>
  (Array.isArray(zones) ? zones : [])
    .map((zone) => ({
      ...zone,
      points: Array.isArray(zone?.points)
        ? zone.points
            .map((point) => ({
              x: Number(point?.x),
              y: Number(point?.y),
            }))
            .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
        : [],
    }))
    .filter((zone) => zone.points.length >= 3);

export const resolveSurfaceAtPoint = (point, zones = SURFACE_ZONE_PRESETS) => {
  const normalizedZones = normalizeSurfaceZones(zones);
  for (let index = normalizedZones.length - 1; index >= 0; index -= 1) {
    const zone = normalizedZones[index];
    if (pointInPolygon(point, zone.points)) {
      return {
        zone,
        profile: getSurfaceProfileByKey(zone.surfaceKey),
      };
    }
  }
  return {
    zone: null,
    profile: SURFACE_PROFILES.neutral,
  };
};

const calcSpeedFactor = (profile, speedMps) => {
  const nominal = Math.max(0.05, safeNumber(profile.nominalSpeedMps, DEFAULT_SPEED_MPS));
  const requested = Math.max(0.01, safeNumber(speedMps, DEFAULT_SPEED_MPS));
  const speedRatio = requested / nominal;
  const overSpeed = Math.max(0, requested - profile.maxSpeedMps);

  if (speedRatio <= 1) {
    return clamp(1 - (1 - speedRatio) * 0.08, 0.9, 1);
  }

  const ratioPenalty = Math.pow(speedRatio - 1, 2) * profile.speedPenaltyGain;
  const overSpeedPenalty = overSpeed * 1.1;
  return 1 + ratioPenalty + overSpeedPenalty;
};

const calcPayloadFactor = (profile, payloadKg) => {
  const payload = Math.max(0, safeNumber(payloadKg, DEFAULT_PAYLOAD_KG));
  return 1 + payload * profile.payloadPenaltyPerKg;
};

export const describeSurfaceRuntime = (
  profileOrKey,
  {
    speedMps = DEFAULT_SPEED_MPS,
    payloadKg = DEFAULT_PAYLOAD_KG,
  } = {}
) => {
  const resolvedProfile =
    typeof profileOrKey === "string"
      ? getSurfaceProfileByKey(profileOrKey)
      : getSurfaceProfileByKey(profileOrKey?.key);
  const requestedSpeedMps = Math.max(0.01, safeNumber(speedMps, DEFAULT_SPEED_MPS));
  const speedFactor = calcSpeedFactor(resolvedProfile, requestedSpeedMps);
  const payloadFactor = calcPayloadFactor(resolvedProfile, payloadKg);
  const effectiveSpeedMps = clamp(
    Math.min(requestedSpeedMps, resolvedProfile.maxSpeedMps),
    0.01,
    1.2
  );

  return {
    requestedSpeedMps,
    effectiveSpeedMps,
    surfaceMaxSpeedMps: resolvedProfile.maxSpeedMps,
    speedFactor,
    payloadFactor,
    energyMultiplier: speedFactor * payloadFactor,
  };
};

export const estimateRouteEnergy = (
  route,
  {
    surfaceZones = SURFACE_ZONE_PRESETS,
    speedMps = DEFAULT_SPEED_MPS,
    payloadKg = DEFAULT_PAYLOAD_KG,
    includeTurnPenalty = true,
  } = {}
) => {
  if (!Array.isArray(route) || route.length < 2) {
    return {
      totalEnergy: 0,
      distanceMeters: 0,
      estimatedTimeSec: 0,
      averageSlipRisk: 0,
      limitingMaxSpeedMps: SURFACE_PROFILES.neutral.maxSpeedMps,
      segmentCount: 0,
    };
  }

  let totalEnergy = 0;
  let distanceMeters = 0;
  let estimatedTimeSec = 0;
  let weightedSlipRisk = 0;
  let limitingMaxSpeedMps = Number.POSITIVE_INFINITY;

  for (let index = 1; index < route.length; index += 1) {
    const from = route[index - 1];
    const to = route[index];
    const segmentDistance = dist(from, to);
    if (!Number.isFinite(segmentDistance) || segmentDistance <= EPS) continue;

    const sample = {
      x: (from.x + to.x) * 0.5,
      y: (from.y + to.y) * 0.5,
    };
    const { profile } = resolveSurfaceAtPoint(sample, surfaceZones);
    const speedFactor = calcSpeedFactor(profile, speedMps);
    const payloadFactor = calcPayloadFactor(profile, payloadKg);
    const segmentEnergy = segmentDistance * profile.energyPerMeter * speedFactor * payloadFactor;
    const effectiveSpeed = clamp(
      Math.min(safeNumber(speedMps, DEFAULT_SPEED_MPS), profile.maxSpeedMps),
      0.01,
      1.2
    );

    totalEnergy += segmentEnergy;
    distanceMeters += segmentDistance;
    estimatedTimeSec += segmentDistance / effectiveSpeed;
    weightedSlipRisk += profile.slipRisk * segmentDistance;
    limitingMaxSpeedMps = Math.min(limitingMaxSpeedMps, profile.maxSpeedMps);
  }

  if (includeTurnPenalty) {
    for (let index = 1; index < route.length - 1; index += 1) {
      const prev = route[index - 1];
      const current = route[index];
      const next = route[index + 1];
      const headingA = Math.atan2(current.y - prev.y, current.x - prev.x);
      const headingB = Math.atan2(next.y - current.y, next.x - current.x);
      const turnAngle = Math.abs(normalizeAngle(headingB - headingA));
      if (turnAngle <= EPS) continue;

      const { profile } = resolveSurfaceAtPoint(current, surfaceZones);
      const speedFactor = calcSpeedFactor(profile, speedMps);
      const payloadFactor = calcPayloadFactor(profile, payloadKg);
      totalEnergy += turnAngle * profile.turnEnergyPerRad * speedFactor * payloadFactor;
    }
  }

  const averageSlipRisk = distanceMeters > EPS ? weightedSlipRisk / distanceMeters : 0;
  return {
    totalEnergy,
    distanceMeters,
    estimatedTimeSec,
    averageSlipRisk,
    limitingMaxSpeedMps: Number.isFinite(limitingMaxSpeedMps)
      ? limitingMaxSpeedMps
      : SURFACE_PROFILES.neutral.maxSpeedMps,
    segmentCount: Math.max(0, route.length - 1),
  };
};
