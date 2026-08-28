const COST_EPS = 1e-6;
const FUEL_EPS = 1e-6;
export const FRONTIER_LIMIT = 28;

export const dominates = (left, right) =>
  left.cost <= right.cost + COST_EPS && left.fuel >= right.fuel - FUEL_EPS;

export const pruneDominatedLabels = (labels) => {
  const sorted = [...labels].sort((left, right) =>
    left.cost === right.cost ? right.fuel - left.fuel : left.cost - right.cost
  );
  const pruned = [];
  for (const label of sorted) {
    if (!pruned.some((candidate) => dominates(candidate, label))) pruned.push(label);
  }
  return pruned;
};

export const keepFrontierBounded = (labels, limit = FRONTIER_LIMIT) => {
  const pruned = pruneDominatedLabels(labels);
  if (pruned.length <= limit) return pruned;
  return pruned
    .sort((left, right) => {
      if (left.cost !== right.cost) return left.cost - right.cost;
      if (left.fuel !== right.fuel) return right.fuel - left.fuel;
      return left.stationStops - right.stationStops;
    })
    .slice(0, limit);
};
