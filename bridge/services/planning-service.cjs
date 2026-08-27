const {
  clampInt,
  normalizeNumber,
  resolveAlgorithmKey,
  resolveTaskKey,
  sanitizeNativeParams,
  validatePoints,
} = require("../protocol/validation.cjs");

const createPlanningService = ({ nativeSolver, maxConcurrentRuns = 2 }) => {
  let activeRuns = 0;

  const solve = async (request) => {
    const { unwrapPlanningRequest } = await import("../../shared/contracts/index.js");
    const payload = request?.type === "planning.request"
      ? unwrapPlanningRequest(request)
      : request;
    const points = validatePoints(payload?.points);
    const algorithm = resolveAlgorithmKey(payload?.algorithm?.key);
    const params = sanitizeNativeParams(algorithm, payload?.algorithm?.params);
    const task = resolveTaskKey(payload?.task);
    const seed = clampInt(normalizeNumber(payload?.seed, 1337), 1, 2147483647);
    if (activeRuns >= maxConcurrentRuns) {
      const error = new Error("Solver is busy. Try again shortly.");
      error.statusCode = 503;
      throw error;
    }

    activeRuns += 1;
    try {
      const solved = await nativeSolver.run({ points, algorithmKey: algorithm, params, taskKey: task, seed });
      return {
        ok: true,
        task,
        algorithm,
        length: solved.length,
        closed: solved.closed,
        order: solved.order,
        route: solved.route,
      };
    } finally {
      activeRuns -= 1;
    }
  };

  return { solve };
};

module.exports = { createPlanningService };
