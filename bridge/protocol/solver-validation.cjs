const {
  clamp,
  clampInt,
  normalizeNumber,
} = require("./number-normalization.cjs");

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

module.exports = {
  resolveAlgorithmKey,
  resolveTaskKey,
  sanitizeNativeParams,
};
