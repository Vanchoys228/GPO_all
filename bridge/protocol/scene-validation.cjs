const { createHash } = require("crypto");
const { validatePoints, validatePolygons, validateSurfaceZones } = require("./route-validation.cjs");
const invalid = message => Object.assign(new Error(message), { statusCode: 400 });
const normalizeScene = (raw) => {
  if (!raw || typeof raw !== "object" || Array.isArray(raw)) throw invalid("A scene snapshot is required.");
  const motion = raw.motion || {};
  const number = (key, fallback, min, max) => {
    const value = motion[key] === undefined ? fallback : Number(motion[key]);
    if (!Number.isFinite(value) || value < min || value > max) throw invalid(`Invalid scene motion.${key}.`);
    return value;
  };
  try {
    return {
      polygons: validatePolygons(raw.polygons ?? []),
      surfaceZones: validateSurfaceZones(raw.surfaceZones ?? []),
      chargingStations: validatePoints(raw.chargingStations ?? []),
      motion: { cruiseSpeedMps: number("cruiseSpeedMps", 0.22, 0.05, 0.8), payloadKg: number("payloadKg", 0, 0, 500), batteryRange: number("batteryRange", 100, 1, 100000) },
    };
  } catch (error) { error.statusCode = 400; throw error; }
};
const sceneRevision = scene => createHash("sha256").update(JSON.stringify(scene)).digest("hex");
module.exports = { normalizeScene, sceneRevision };
