const EPS = 1e-9;
const DEFAULT_SPEED_MPS = 0.22;
const DEFAULT_PAYLOAD_KG = 0;
const CONTROLLER_MAX_LINEAR_SPEED_MPS = 0.8;
const CONTROLLER_MIN_LINEAR_SPEED_MPS = 0.045;
const CONTROLLER_POSITION_TOLERANCE_M = 0.05;
const CONTROLLER_TRACK_SLOW_RADIUS_M = 0.22;
const CONTROLLER_NEAR_TARGET_MAX_SPEED_MPS = 0.24;
const CONTROLLER_ANGULAR_SPEED_RAD_S = 1.6;
const CONTROLLER_TURN_TIME_FACTOR = 0.55;
const CONTROLLER_TIME_STEP_M = 0.04;
const CONTROLLER_TIME_CALIBRATION_FACTOR = 1.146;

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
    .filter((zone) => zone.points.length >= 3 && zone.closed !== false);

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

const estimateControllerSpeedAtDistance = (distanceToTarget, requestedSpeedMps) => {
  const requested = clamp(
    safeNumber(requestedSpeedMps, DEFAULT_SPEED_MPS),
    CONTROLLER_MIN_LINEAR_SPEED_MPS,
    CONTROLLER_MAX_LINEAR_SPEED_MPS
  );
  if (distanceToTarget < CONTROLLER_TRACK_SLOW_RADIUS_M) {
    const nearTargetCap = clamp(requested * 0.36, 0.14, CONTROLLER_NEAR_TARGET_MAX_SPEED_MPS);
    return clamp(
      distanceToTarget * 1.05,
      Math.min(CONTROLLER_MIN_LINEAR_SPEED_MPS, requested),
      Math.min(nearTargetCap, requested)
    );
  }
  return clamp(
    distanceToTarget * 1.18,
    Math.min(CONTROLLER_MIN_LINEAR_SPEED_MPS, requested),
    requested
  );
};

const estimateControllerSegmentTime = (segmentDistance, requestedSpeedMps) => {
  if (!Number.isFinite(segmentDistance) || segmentDistance <= CONTROLLER_POSITION_TOLERANCE_M) {
    return 0;
  }

  let remaining = segmentDistance;
  let timeSec = 0;
  while (remaining > CONTROLLER_POSITION_TOLERANCE_M + EPS) {
    const step = Math.min(CONTROLLER_TIME_STEP_M, remaining - CONTROLLER_POSITION_TOLERANCE_M);
    const speed = estimateControllerSpeedAtDistance(remaining, requestedSpeedMps);
    timeSec += step / Math.max(speed, 0.01);
    remaining -= step;
  }

  return timeSec;
};

const estimateControllerTurnTime = (turnAngleRad) => {
  if (!Number.isFinite(turnAngleRad) || turnAngleRad <= EPS) return 0;
  return (turnAngleRad / CONTROLLER_ANGULAR_SPEED_RAD_S) * CONTROLLER_TURN_TIME_FACTOR;
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

    totalEnergy += segmentEnergy;
    distanceMeters += segmentDistance;
    estimatedTimeSec += estimateControllerSegmentTime(segmentDistance, speedMps);
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
      estimatedTimeSec += estimateControllerTurnTime(turnAngle);
    }
  }

  const averageSlipRisk = distanceMeters > EPS ? weightedSlipRisk / distanceMeters : 0;
  return {
    totalEnergy,
    distanceMeters,
    estimatedTimeSec: estimatedTimeSec * CONTROLLER_TIME_CALIBRATION_FACTOR,
    averageSlipRisk,
    limitingMaxSpeedMps: Number.isFinite(limitingMaxSpeedMps)
      ? limitingMaxSpeedMps
      : SURFACE_PROFILES.neutral.maxSpeedMps,
    segmentCount: Math.max(0, route.length - 1),
  };
};

const formatSigned = (value, digits = 1) => {
  if (!Number.isFinite(value) || Math.abs(value) < 0.05) return "0";
  const sign = value > 0 ? "+" : "-";
  return `${sign}${Math.abs(value).toFixed(digits)}`;
};

const formatEnergyImpact = (value, totalEnergy) => {
  const percentBase = Math.max(Math.abs(totalEnergy), 1);
  const percent = (value / percentBase) * 100;
  return `${formatSigned(value, 1)} ед. (${formatSigned(percent, 1)}%)`;
};

const countClosedSurfaceZones = (surfaceZones) =>
  (Array.isArray(surfaceZones) ? surfaceZones : []).filter(
    (zone) => zone?.closed !== false && Array.isArray(zone?.points) && zone.points.length >= 3
  ).length;

export const analyzeRouteInfluence = (
  route,
  {
    surfaceZones = SURFACE_ZONE_PRESETS,
    speedMps = DEFAULT_SPEED_MPS,
    payloadKg = DEFAULT_PAYLOAD_KG,
    stationStopCount = 0,
    plannedTimeSec = 0,
    actualTimeSec = null,
    avoidanceTimeSec = 0,
  } = {}
) => {
  if (!Array.isArray(route) || route.length < 2) return [];

  const actual = estimateRouteEnergy(route, {
    surfaceZones,
    speedMps,
    payloadKg,
  });
  const neutralSameInputs = estimateRouteEnergy(route, {
    surfaceZones: [],
    speedMps,
    payloadKg,
  });
  const noPayload = estimateRouteEnergy(route, {
    surfaceZones,
    speedMps,
    payloadKg: 0,
  });
  const defaultSpeed = estimateRouteEnergy(route, {
    surfaceZones,
    speedMps: DEFAULT_SPEED_MPS,
    payloadKg,
  });
  const noTurns = estimateRouteEnergy(route, {
    surfaceZones,
    speedMps,
    payloadKg,
    includeTurnPenalty: false,
  });
  const neutralBase = estimateRouteEnergy(route, {
    surfaceZones: [],
    speedMps: DEFAULT_SPEED_MPS,
    payloadKg: 0,
    includeTurnPenalty: false,
  });

  const rows = [
    {
      key: "distance",
      label: "Длина маршрута",
      value: `${actual.distanceMeters.toFixed(1)} м`,
      impact: `${neutralBase.totalEnergy.toFixed(1)} ед. базового расхода`,
    },
    {
      key: "surfaces",
      label: "Типы покрытий",
      value: `${countClosedSurfaceZones(surfaceZones)} зон`,
      impact: formatEnergyImpact(
        actual.totalEnergy - neutralSameInputs.totalEnergy,
        actual.totalEnergy
      ),
    },
    {
      key: "payload",
      label: "Масса груза",
      value: `${Math.max(0, safeNumber(payloadKg, 0)).toFixed(1)} кг`,
      impact: formatEnergyImpact(actual.totalEnergy - noPayload.totalEnergy, actual.totalEnergy),
    },
    {
      key: "speed",
      label: "Заданная скорость",
      value: `${Math.max(0, safeNumber(speedMps, DEFAULT_SPEED_MPS)).toFixed(2)} м/с`,
      impact: formatEnergyImpact(actual.totalEnergy - defaultSpeed.totalEnergy, actual.totalEnergy),
    },
    {
      key: "turns",
      label: "Повороты и манёвры",
      value: `${Math.max(0, route.length - 2)} поворотов`,
      impact: formatEnergyImpact(actual.totalEnergy - noTurns.totalEnergy, actual.totalEnergy),
    },
    {
      key: "slip",
      label: "Риск проскальзывания",
      value: `${(actual.averageSlipRisk * 100).toFixed(1)}%`,
      impact:
        actual.averageSlipRisk > 0.16
          ? "нужна сниженная скорость"
          : actual.averageSlipRisk > 0.07
            ? "умеренное влияние"
            : "низкое влияние",
    },
    {
      key: "charging",
      label: "Заезды на зарядку",
      value: `${Math.max(0, Number(stationStopCount) || 0)}`,
      impact:
        Number(stationStopCount) > 0
          ? "маршрут удлинён зарядкой"
          : "без влияния",
    },
    {
      key: "avoidance-time",
      label: "Объезд вне маршрута",
      value: `${Math.round(Math.max(0, Number(avoidanceTimeSec) || 0))} сек`,
      impact:
        Number(avoidanceTimeSec) > 0
          ? `+${Math.round(Number(avoidanceTimeSec))} сек к факту`
          : "без внепланового объезда",
    },
  ];

  if (Number.isFinite(actualTimeSec) && actualTimeSec > 0 && Number.isFinite(plannedTimeSec)) {
    const delta = actualTimeSec - plannedTimeSec;
    rows.push({
      key: "actual-time",
      label: "Факт против плана",
      value: `${Math.round(actualTimeSec)} сек`,
      impact: `${formatSigned(delta, 0)} сек`,
    });
  }

  return rows;
};
