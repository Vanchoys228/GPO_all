import { DEFAULT_PAYLOAD_KG, DEFAULT_SPEED_MPS, SURFACE_PROFILES, SURFACE_ZONE_PRESETS, getSurfaceProfileByKey } from "./energyProfiles";
import { resolveSurfaceAtPoint } from "./energySurfaceZones";

const EPS = 1e-9;
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
const safeNumber = (value, fallback) => Number.isFinite(Number(value)) ? Number(value) : fallback;

const normalizeAngle = (angle) => {
  let normalized = angle;
  while (normalized > Math.PI) normalized -= Math.PI * 2;
  while (normalized < -Math.PI) normalized += Math.PI * 2;
  return normalized;
};

const calcSpeedFactor = (profile, speedMps) => {
  const nominal = Math.max(0.05, safeNumber(profile.nominalSpeedMps, DEFAULT_SPEED_MPS));
  const requested = Math.max(0.01, safeNumber(speedMps, DEFAULT_SPEED_MPS));
  const speedRatio = requested / nominal;
  if (speedRatio <= 1) return clamp(1 - (1 - speedRatio) * 0.08, 0.9, 1);
  return 1 + Math.pow(speedRatio - 1, 2) * profile.speedPenaltyGain + Math.max(0, requested - profile.maxSpeedMps) * 1.1;
};

const calcPayloadFactor = (profile, payloadKg) =>
  1 + Math.max(0, safeNumber(payloadKg, DEFAULT_PAYLOAD_KG)) * profile.payloadPenaltyPerKg;

const estimateControllerSpeedAtDistance = (distanceToTarget, requestedSpeedMps) => {
  const requested = clamp(safeNumber(requestedSpeedMps, DEFAULT_SPEED_MPS), CONTROLLER_MIN_LINEAR_SPEED_MPS, CONTROLLER_MAX_LINEAR_SPEED_MPS);
  if (distanceToTarget < CONTROLLER_TRACK_SLOW_RADIUS_M) {
    const nearTargetCap = clamp(requested * 0.36, 0.14, CONTROLLER_NEAR_TARGET_MAX_SPEED_MPS);
    return clamp(distanceToTarget * 1.05, Math.min(CONTROLLER_MIN_LINEAR_SPEED_MPS, requested), Math.min(nearTargetCap, requested));
  }
  return clamp(distanceToTarget * 1.18, Math.min(CONTROLLER_MIN_LINEAR_SPEED_MPS, requested), requested);
};

const estimateControllerSegmentTime = (segmentDistance, requestedSpeedMps) => {
  if (!Number.isFinite(segmentDistance) || segmentDistance <= CONTROLLER_POSITION_TOLERANCE_M) return 0;
  let remaining = segmentDistance;
  let timeSec = 0;
  while (remaining > CONTROLLER_POSITION_TOLERANCE_M + EPS) {
    const step = Math.min(CONTROLLER_TIME_STEP_M, remaining - CONTROLLER_POSITION_TOLERANCE_M);
    timeSec += step / Math.max(estimateControllerSpeedAtDistance(remaining, requestedSpeedMps), 0.01);
    remaining -= step;
  }
  return timeSec;
};

export const describeSurfaceRuntime = (profileOrKey, { speedMps = DEFAULT_SPEED_MPS, payloadKg = DEFAULT_PAYLOAD_KG } = {}) => {
  const profile = getSurfaceProfileByKey(typeof profileOrKey === "string" ? profileOrKey : profileOrKey?.key);
  const requestedSpeedMps = Math.max(0.01, safeNumber(speedMps, DEFAULT_SPEED_MPS));
  const speedFactor = calcSpeedFactor(profile, requestedSpeedMps);
  const payloadFactor = calcPayloadFactor(profile, payloadKg);
  return {
    requestedSpeedMps,
    effectiveSpeedMps: clamp(Math.min(requestedSpeedMps, profile.maxSpeedMps), 0.01, 1.2),
    surfaceMaxSpeedMps: profile.maxSpeedMps,
    speedFactor,
    payloadFactor,
    energyMultiplier: speedFactor * payloadFactor,
  };
};

export const estimateRouteEnergy = (route, {
  surfaceZones = SURFACE_ZONE_PRESETS, speedMps = DEFAULT_SPEED_MPS, payloadKg = DEFAULT_PAYLOAD_KG, includeTurnPenalty = true,
} = {}) => {
  if (!Array.isArray(route) || route.length < 2) {
    return { totalEnergy: 0, distanceMeters: 0, estimatedTimeSec: 0, averageSlipRisk: 0, limitingMaxSpeedMps: SURFACE_PROFILES.neutral.maxSpeedMps, segmentCount: 0 };
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
    const { profile } = resolveSurfaceAtPoint({ x: (from.x + to.x) * 0.5, y: (from.y + to.y) * 0.5 }, surfaceZones);
    const speedFactor = calcSpeedFactor(profile, speedMps);
    const payloadFactor = calcPayloadFactor(profile, payloadKg);
    totalEnergy += segmentDistance * profile.energyPerMeter * speedFactor * payloadFactor;
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
      const turnAngle = Math.abs(normalizeAngle(Math.atan2(next.y - current.y, next.x - current.x) - Math.atan2(current.y - prev.y, current.x - prev.x)));
      if (turnAngle <= EPS) continue;
      const { profile } = resolveSurfaceAtPoint(current, surfaceZones);
      totalEnergy += turnAngle * profile.turnEnergyPerRad * calcSpeedFactor(profile, speedMps) * calcPayloadFactor(profile, payloadKg);
      estimatedTimeSec += (turnAngle / CONTROLLER_ANGULAR_SPEED_RAD_S) * CONTROLLER_TURN_TIME_FACTOR;
    }
  }
  return {
    totalEnergy,
    distanceMeters,
    estimatedTimeSec: estimatedTimeSec * CONTROLLER_TIME_CALIBRATION_FACTOR,
    averageSlipRisk: distanceMeters > EPS ? weightedSlipRisk / distanceMeters : 0,
    limitingMaxSpeedMps: Number.isFinite(limitingMaxSpeedMps) ? limitingMaxSpeedMps : SURFACE_PROFILES.neutral.maxSpeedMps,
    segmentCount: Math.max(0, route.length - 1),
  };
};
