const fs = require("fs");
const path = require("path");
const {
  createMotionProfileText,
  createRuntimeCommandText,
  sanitizeMotionProfile,
  sanitizeRuntimeObstacle,
} = require("./web-state-serializers.cjs");
const {
  resolveAlgorithmKey,
  resolveTaskKey,
} = require("../protocol/solver-validation.cjs");
const {
  validatePoints,
  validatePolygons,
  validateSurfaceZones,
} = require("../protocol/route-validation.cjs");

const toDegrees = (radians) => (radians * 180) / Math.PI;

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
    await fsp.writeFile(paths.motionProfile, createMotionProfileText(motionProfile));
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
    const commandText = createRuntimeCommandText(payload);
    if (payload?.type === "start_mapping_survey") {
      const motion = sanitizeMotionProfile(payload?.motion);
      await Promise.all([
        fsp.writeFile(paths.runtimeCommand, commandText),
        writeMotionProfile(motion),
      ]);
      return;
    }

    await fsp.writeFile(paths.runtimeCommand, commandText);
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
