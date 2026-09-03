const SURFACE_KEYS = new Set(["neutral", "rough", "slippery"]);
const MAX_ROUTE_POINTS = 1000;

const createValidationError = (message) => {
  const error = new Error(message);
  error.statusCode = 400;
  return error;
};

const validatePoints = (points) => {
  if (!Array.isArray(points)) {
    throw createValidationError("Points payload must be an array.");
  }
  if (points.length > MAX_ROUTE_POINTS) {
    throw createValidationError(`A route may contain at most ${MAX_ROUTE_POINTS} points.`);
  }

  return points.map((point) => {
    const x = Number(point?.x);
    const y = Number(point?.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      throw createValidationError("Every route point must contain finite x and y.");
    }
    return { x, y };
  });
};

const validatePolygons = (zones) => {
  if (!Array.isArray(zones)) {
    throw new Error("Limit zones payload must be an array.");
  }

  return zones.map((zone, index) => {
    const id =
      typeof zone?.id === "string" && zone.id.trim()
        ? zone.id.trim()
        : `zone-${index + 1}`;
    const name =
      typeof zone?.name === "string" && zone.name.trim()
        ? zone.name.trim()
        : `Zone ${index + 1}`;
    const points = validatePoints(zone?.points || []);
    if (points.length < 3) {
      throw new Error("Every limit zone must contain at least three points.");
    }
    return { id, name, points };
  });
};

const validateSurfaceZones = (zones) => {
  if (!Array.isArray(zones)) {
    throw new Error("Surface zones payload must be an array.");
  }

  return zones.map((zone, index) => {
    const id =
      typeof zone?.id === "string" && zone.id.trim()
        ? zone.id.trim()
        : `surface-zone-${index + 1}`;
    const name =
      typeof zone?.name === "string" && zone.name.trim()
        ? zone.name.trim()
        : `Surface ${index + 1}`;
    const surfaceKey = SURFACE_KEYS.has(zone?.surfaceKey) ? zone.surfaceKey : "neutral";
    const points = validatePoints(zone?.points || []);
    if (points.length < 3) {
      throw new Error("Every surface zone must contain at least three points.");
    }
    return { id, name, surfaceKey, points };
  });
};

module.exports = {
  MAX_ROUTE_POINTS,
  validatePoints,
  validatePolygons,
  validateSurfaceZones,
};
