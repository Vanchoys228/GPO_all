const DEFAULT_GA_TABU_PARAMS = {
  population_size: 90,
  generations: 260,
  mutation_rate: 0.18,
  crossover_rate: 0.9,
  tabu_iterations: 24,
};

const DEFAULT_ANNEALING_PARAMS = {
  pool_size: 48,
  max_iterations: 5000,
  minimum_temperature: 0.001,
  cooling_rate: 0.996,
  neighborhood_strength: 2.5,
};

const DEFAULT_SCATTER_PARAMS = {
  population_size: 70,
  refset_ratio: 0.18,
  max_iterations: 130,
  mutation_rate: 0.2,
  local_steps: 8,
};

const DEFAULT_CUCKOO_PARAMS = {
  nests: 30,
  discovery_probability: 0.25,
  max_iterations: 200,
  alpha: 0.12,
  beta: 1.5,
};

const SURFACE_KEYS = new Set(["neutral", "rough", "slippery"]);
const MAX_ROUTE_POINTS = 1000;

const createValidationError = (message) => {
  const error = new Error(message);
  error.statusCode = 400;
  return error;
};

const clamp = (value, min, max) => Math.max(min, Math.min(value, max));

const clampInt = (value, min, max) =>
  Math.round(clamp(Number.isFinite(value) ? value : min, min, max));

const normalizeNumber = (value, fallback) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

const resolveAlgorithmKey = (algorithmKey) => {
  if (algorithmKey === "genetik") return "ga_tabu";
  if (algorithmKey === "annealing") return "otshig";
  if (algorithmKey === "scatter") return "rasseivanie";
  return algorithmKey || "ga_tabu";
};

const resolveTaskKey = (taskKey) => {
  if (
    taskKey === "tsp" ||
    taskKey === "hamiltonian_chain" ||
    taskKey === "shortest_route"
  ) {
    return taskKey;
  }
  return "tsp";
};

const sanitizeNativeParams = (algorithmKey, rawParams) => {
  const resolvedKey = resolveAlgorithmKey(algorithmKey);
  const params = rawParams || {};

  if (resolvedKey === "ga_tabu") {
    const base = { ...DEFAULT_GA_TABU_PARAMS, ...params };
    return {
      nests: clampInt(base.population_size, 8, 1200),
      pa: clamp(normalizeNumber(base.mutation_rate, DEFAULT_GA_TABU_PARAMS.mutation_rate), 0, 1),
      max_iter: clampInt(base.generations, 1, 10000),
      alpha: clamp(normalizeNumber(base.crossover_rate, DEFAULT_GA_TABU_PARAMS.crossover_rate), 0, 1),
      beta: clamp(normalizeNumber(base.tabu_iterations, DEFAULT_GA_TABU_PARAMS.tabu_iterations), 1, 80),
    };
  }

  if (resolvedKey === "otshig") {
    const base = { ...DEFAULT_ANNEALING_PARAMS, ...params };
    return {
      nests: clampInt(base.pool_size, 8, 200),
      pa: clamp(
        normalizeNumber(base.minimum_temperature, DEFAULT_ANNEALING_PARAMS.minimum_temperature),
        0.000001,
        1
      ),
      max_iter: clampInt(base.max_iterations, 1, 200000),
      alpha: clamp(
        normalizeNumber(base.cooling_rate, DEFAULT_ANNEALING_PARAMS.cooling_rate),
        0.9,
        0.99999
      ),
      beta: clamp(
        normalizeNumber(base.neighborhood_strength, DEFAULT_ANNEALING_PARAMS.neighborhood_strength),
        1,
        12
      ),
    };
  }

  if (resolvedKey === "rasseivanie") {
    const base = { ...DEFAULT_SCATTER_PARAMS, ...params };
    return {
      nests: clampInt(base.population_size, 10, 1200),
      pa: clamp(normalizeNumber(base.refset_ratio, DEFAULT_SCATTER_PARAMS.refset_ratio), 0.05, 0.5),
      max_iter: clampInt(base.max_iterations, 1, 3000),
      alpha: clamp(
        normalizeNumber(base.mutation_rate, DEFAULT_SCATTER_PARAMS.mutation_rate) * 2,
        0,
        1
      ),
      beta: clamp(normalizeNumber(base.local_steps, DEFAULT_SCATTER_PARAMS.local_steps), 2, 30),
    };
  }

  const base = { ...DEFAULT_CUCKOO_PARAMS, ...params };
  return {
    nests: clampInt(base.nests, 5, 250),
    pa: clamp(
      normalizeNumber(base.discovery_probability, DEFAULT_CUCKOO_PARAMS.discovery_probability),
      0.01,
      0.9
    ),
    max_iter: clampInt(base.max_iterations, 1, 5000),
    alpha: clamp(normalizeNumber(base.alpha, DEFAULT_CUCKOO_PARAMS.alpha), 0.001, 2),
    beta: clamp(normalizeNumber(base.beta, DEFAULT_CUCKOO_PARAMS.beta), 1.1, 1.99),
  };
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

const sanitizeMappingSurveyMode = (rawMode) => {
  const mode = String(rawMode || "snake").trim().toLowerCase();
  if (["snake", "double"].includes(mode)) return mode;
  return "snake";
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
  clamp,
  clampInt,
  normalizeNumber,
  resolveAlgorithmKey,
  resolveTaskKey,
  sanitizeMappingSurveyMode,
  sanitizeNativeParams,
  validatePoints,
  validatePolygons,
  validateSurfaceZones,
};
