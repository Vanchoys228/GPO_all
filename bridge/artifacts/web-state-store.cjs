const fs = require("fs");
const path = require("path");
const {
  clamp,
  normalizeNumber,
  resolveAlgorithmKey,
  resolveTaskKey,
  sanitizeMappingSurveyMode,
  validatePoints,
  validatePolygons,
  validateSurfaceZones,
} = require("../protocol/validation.cjs");

const DEFAULT_MOTION_PROFILE = {
  cruiseSpeedMps: 0.22,
  payloadKg: 0,
  batteryRange: 100,
};

const toDegrees = (radians) => (radians * 180) / Math.PI;

const sanitizeMotionProfile = (motion) => {
  const profile = motion || {};
  return {
    cruiseSpeedMps: clamp(
      normalizeNumber(profile.cruiseSpeedMps, DEFAULT_MOTION_PROFILE.cruiseSpeedMps),
      0.05,
      0.8
    ),
    payloadKg: clamp(
      normalizeNumber(profile.payloadKg, DEFAULT_MOTION_PROFILE.payloadKg),
      0,
      500
    ),
    batteryRange: clamp(
      normalizeNumber(profile.batteryRange, DEFAULT_MOTION_PROFILE.batteryRange),
      1,
      100000
    ),
  };
};

const sanitizeRuntimeObstacle = (rawObstacle) => {
  const obstacle = rawObstacle || {};
  return {
    x: clamp(normalizeNumber(obstacle.x, 0), -21.5, 21.5),
    y: clamp(normalizeNumber(obstacle.y, 0), -16.5, 16.5),
    sizeX: clamp(normalizeNumber(obstacle.sizeX, 0.8), 0.2, 3.5),
    sizeY: clamp(normalizeNumber(obstacle.sizeY, 0.8), 0.2, 3.5),
    height: clamp(normalizeNumber(obstacle.height, 0.6), 0.12, 2.8),
  };
};

const normalizeCommandId = (rawCommandId) => {
  const value = Number(rawCommandId);
  return Number.isFinite(value) && value > 0 ? Math.trunc(value) : Date.now();
};

const createWebStateStore = ({ coordinateContract, stateDir }) => {
  const fsp = fs.promises;
  const paths = {
    routeJson: path.join(stateDir, "route.json"),
    routeCsv: path.join(stateDir, "route.csv"),
    limitZonesJson: path.join(stateDir, "limit_zones.json"),
    limitZonesTxt: path.join(stateDir, "limit_zones.txt"),
    surfaceZonesJson: path.join(stateDir, "surface_zones.json"),
    surfaceZonesTxt: path.join(stateDir, "surface_zones.txt"),
    motionProfile: path.join(stateDir, "motion_profile.txt"),
    runtimeCommand: path.join(stateDir, "runtime_command.txt"),
  };
  const routeCsvHeader = coordinateContract.routeCsv.header.join(",");
  let ensureStateDirPromise = null;

  const ensureStateDir = () => {
    if (!ensureStateDirPromise) {
      ensureStateDirPromise = fsp.mkdir(stateDir, { recursive: true });
    }
    return ensureStateDirPromise;
  };

  const writeMotionProfile = async (motion) => {
    await ensureStateDir();
    const motionProfile = sanitizeMotionProfile(motion);
    const lines = [
      `cruise_speed_mps ${motionProfile.cruiseSpeedMps}`,
      `payload_kg ${motionProfile.payloadKg}`,
      `battery_range ${motionProfile.batteryRange}`,
    ];
    await fsp.writeFile(paths.motionProfile, `${lines.join("\n")}\n`);
    return motionProfile;
  };

  const writeRoute = async (payload) => {
    await ensureStateDir();
    const route = validatePoints(payload?.route || []);
    const task = resolveTaskKey(payload?.algorithm?.task);
    const algorithmKey = resolveAlgorithmKey(payload?.algorithm?.key);
    const params = payload?.algorithm?.params || {};
    const motion = sanitizeMotionProfile(payload?.motion);
    const routeJson = {
      type: "route",
      coordinateContractVersion: coordinateContract.version,
      createdAt: new Date().toISOString(),
      task,
      algorithm: { key: algorithmKey, params },
      motion,
      route,
    };
    const csvLines = [routeCsvHeader];
    for (let index = 0; index < route.length; index += 1) {
      const point = route[index];
      const previous = index === 0 ? { x: 0, y: 0 } : route[index - 1];
      const headingDeg = toDegrees(Math.atan2(point.y - previous.y, point.x - previous.x));
      csvLines.push(`${point.x},${point.y},${headingDeg}`);
    }
    await Promise.all([
      fsp.writeFile(paths.routeJson, JSON.stringify(routeJson, null, 2)),
      fsp.writeFile(paths.routeCsv, `${csvLines.join("\n")}\n`),
      writeMotionProfile(motion),
    ]);
  };

  const writeLimitZones = async (payload) => {
    await ensureStateDir();
    const zones = validatePolygons(payload?.zones || []);
    const zonesJson = {
      type: "limit_zones",
      coordinateContractVersion: coordinateContract.version,
      createdAt: new Date().toISOString(),
      zones,
    };
    const textLines = [`zone_count ${zones.length}`];
    for (const zone of zones) {
      textLines.push(`zone ${zone.points.length}`);
      for (const point of zone.points) textLines.push(`${point.x} ${point.y}`);
    }
    await Promise.all([
      fsp.writeFile(paths.limitZonesJson, JSON.stringify(zonesJson, null, 2)),
      fsp.writeFile(paths.limitZonesTxt, `${textLines.join("\n")}\n`),
    ]);
  };

  const writeSurfaceZones = async (payload) => {
    await ensureStateDir();
    const zones = validateSurfaceZones(payload?.zones || []);
    const zonesJson = {
      type: "surface_zones",
      coordinateContractVersion: coordinateContract.version,
      createdAt: new Date().toISOString(),
      zones,
    };
    const textLines = [`surface_zone_count ${zones.length}`];
    for (const zone of zones) {
      textLines.push(`surface_zone ${zone.points.length} ${zone.surfaceKey} ${zone.id}`);
      for (const point of zone.points) textLines.push(`${point.x} ${point.y}`);
    }
    await Promise.all([
      fsp.writeFile(paths.surfaceZonesJson, JSON.stringify(zonesJson, null, 2)),
      fsp.writeFile(paths.surfaceZonesTxt, `${textLines.join("\n")}\n`),
    ]);
  };

  const writeRuntimeCommand = async (payload) => {
    await ensureStateDir();
    const commandId = normalizeCommandId(payload?.commandId);
    if (payload?.type === "start_mapping_survey") {
      const clearMap = payload?.clearMap === undefined ? true : Boolean(payload.clearMap);
      const mode = sanitizeMappingSurveyMode(payload?.mode);
      const field = payload?.field || {};
      const motion = sanitizeMotionProfile(payload?.motion);
      const minX = normalizeNumber(field.minX, -22);
      const maxX = normalizeNumber(field.maxX, 22);
      const minY = normalizeNumber(field.minY, -17);
      const maxY = normalizeNumber(field.maxY, 17);
      const lines = [
        `id ${commandId}`,
        "type start_mapping_survey",
        `clear_map ${clearMap ? 1 : 0}`,
        `mode ${mode}`,
        `survey_speed_mps ${motion.cruiseSpeedMps}`,
        `field_min_x ${Math.min(minX, maxX)}`,
        `field_max_x ${Math.max(minX, maxX)}`,
        `field_min_y ${Math.min(minY, maxY)}`,
        `field_max_y ${Math.max(minY, maxY)}`,
      ];
      await Promise.all([
        fsp.writeFile(paths.runtimeCommand, `${lines.join("\n")}\n`),
        writeMotionProfile(motion),
      ]);
      return;
    }

    const obstacle = sanitizeRuntimeObstacle(payload?.obstacle);
    const lines = [
      `id ${commandId}`,
      "type spawn_obstacle",
      `x ${obstacle.x}`,
      `y ${obstacle.y}`,
      `size_x ${obstacle.sizeX}`,
      `size_y ${obstacle.sizeY}`,
      `height ${obstacle.height}`,
    ];
    await fsp.writeFile(paths.runtimeCommand, `${lines.join("\n")}\n`);
  };

  return {
    ensureStateDir,
    paths,
    writeLimitZones,
    writeMotionProfile,
    writeRoute,
    writeRuntimeCommand,
    writeSurfaceZones,
  };
};

module.exports = {
  createWebStateStore,
  sanitizeMotionProfile,
  sanitizeRuntimeObstacle,
};
